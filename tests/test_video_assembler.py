import unittest

from video_protocol import (
    UINT32_MAX,
    VideoFrameAssembler,
    encode_video_fragment,
    fragment_count_for_size,
)


def frame_datagrams(frame_id, frame):
    return [
        encode_video_fragment(frame_id, index, frame)
        for index in range(fragment_count_for_size(len(frame)))
    ]


class VideoFrameAssemblerTests(unittest.TestCase):
    def test_reassembles_fragments_received_out_of_order(self):
        frame = bytes(index % 256 for index in range(2500))
        datagrams = frame_datagrams(100, frame)
        assembler = VideoFrameAssembler()

        self.assertIsNone(assembler.add_datagram(datagrams[2]))
        self.assertIsNone(assembler.add_datagram(datagrams[0]))

        result = assembler.add_datagram(datagrams[1])

        self.assertEqual(result, (100, frame))

    def test_identical_duplicate_is_ignored(self):
        frame = b"x" * 2000
        datagrams = frame_datagrams(20, frame)
        assembler = VideoFrameAssembler()

        self.assertIsNone(assembler.add_datagram(datagrams[0]))
        self.assertIsNone(assembler.add_datagram(datagrams[0]))

        result = assembler.add_datagram(datagrams[1])

        self.assertEqual(result, (20, frame))

    def test_newer_frame_discards_incomplete_older_frame(self):
        old_frame = b"a" * 2000
        new_frame = b"b" * 2000
        old_datagrams = frame_datagrams(30, old_frame)
        new_datagrams = frame_datagrams(31, new_frame)
        assembler = VideoFrameAssembler()

        self.assertIsNone(assembler.add_datagram(old_datagrams[0]))
        self.assertIsNone(assembler.add_datagram(new_datagrams[0]))

        result = assembler.add_datagram(new_datagrams[1])

        self.assertEqual(result, (31, new_frame))
        self.assertIsNone(assembler.add_datagram(old_datagrams[1]))

    def test_timed_out_frame_cannot_restart(self):
        old_frame = b"a" * 2000
        new_frame = b"b" * 2000
        old_datagrams = frame_datagrams(40, old_frame)
        new_datagrams = frame_datagrams(41, new_frame)
        assembler = VideoFrameAssembler(timeout_seconds=0.5)

        self.assertIsNone(
            assembler.add_datagram(old_datagrams[0], now=0.0)
        )
        self.assertIsNone(
            assembler.add_datagram(old_datagrams[1], now=1.0)
        )

        self.assertIsNone(
            assembler.add_datagram(new_datagrams[0], now=1.1)
        )
        result = assembler.add_datagram(
            new_datagrams[1],
            now=1.2,
        )

        self.assertEqual(result, (41, new_frame))

    def test_frame_id_wraparound_is_treated_as_newer(self):
        assembler = VideoFrameAssembler()
        first = encode_video_fragment(UINT32_MAX, 0, b"old")
        wrapped = encode_video_fragment(0, 0, b"new")

        self.assertEqual(
            assembler.add_datagram(first),
            (UINT32_MAX, b"old"),
        )
        self.assertEqual(
            assembler.add_datagram(wrapped),
            (0, b"new"),
        )


    def test_new_session_accepts_restarted_frame_ids(self):
        assembler = VideoFrameAssembler()

        old = encode_video_fragment(
            900,
            0,
            b"old",
            session_id=10,
        )
        restarted = encode_video_fragment(
            0,
            0,
            b"new",
            session_id=11,
        )
        delayed_old = encode_video_fragment(
            901,
            0,
            b"delayed",
            session_id=10,
        )

        self.assertEqual(
            assembler.add_datagram(old),
            (900, b"old"),
        )
        self.assertEqual(
            assembler.add_datagram(restarted),
            (0, b"new"),
        )
        self.assertIsNone(
            assembler.add_datagram(delayed_old)
        )


if __name__ == "__main__":
    unittest.main()
