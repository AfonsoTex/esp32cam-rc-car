import unittest

from video_protocol import (
    UINT32_MAX,
    VIDEO_DATAGRAM_MAX_SIZE,
    VIDEO_FRAGMENT_PAYLOAD_SIZE,
    VIDEO_FRAME_MAX_SIZE,
    VIDEO_HEADER,
    VIDEO_HEADER_SIZE,
    VIDEO_MAGIC,
    encode_video_fragment,
    fragment_count_for_size,
    parse_video_fragment,
)


class VideoProtocolTests(unittest.TestCase):
    def test_protocol_sizes_are_safe_for_udp(self):
        self.assertEqual(VIDEO_HEADER_SIZE, 20)
        self.assertEqual(VIDEO_DATAGRAM_MAX_SIZE, 1200)
        self.assertEqual(VIDEO_FRAGMENT_PAYLOAD_SIZE, 1180)

    def test_single_fragment_round_trip(self):
        frame = b"jpeg-data"

        datagram = encode_video_fragment(42, 0, frame)
        parsed = parse_video_fragment(datagram)

        self.assertEqual(
            parsed,
            (1, 42, 0, 1, len(frame), frame),
        )

    def test_large_frame_is_split_into_expected_fragments(self):
        frame = bytes(index % 256 for index in range(2500))
        count = fragment_count_for_size(len(frame))

        self.assertEqual(count, 3)

        datagrams = [
            encode_video_fragment(7, index, frame)
            for index in range(count)
        ]

        self.assertEqual(len(datagrams[0]), 1200)
        self.assertEqual(len(datagrams[1]), 1200)
        self.assertEqual(len(datagrams[2]), 160)

        rebuilt = b"".join(
            parse_video_fragment(datagram)[5]
            for datagram in datagrams
        )

        self.assertEqual(rebuilt, frame)

    def test_malformed_datagrams_are_rejected(self):
        frame = b"valid-jpeg"
        valid = encode_video_fragment(5, 0, frame)

        self.assertIsNone(parse_video_fragment(b""))
        self.assertIsNone(parse_video_fragment(valid[:VIDEO_HEADER_SIZE]))
        self.assertIsNone(parse_video_fragment(b"BAD!" + valid[4:]))
        self.assertIsNone(parse_video_fragment(valid[:-1]))
        self.assertIsNone(
            parse_video_fragment(
                b"x" * (VIDEO_DATAGRAM_MAX_SIZE + 1)
            )
        )

        wrong_count = (
            VIDEO_HEADER.pack(
                VIDEO_MAGIC,
                1,
                5,
                0,
                2,
                len(frame),
            )
            + frame
        )
        self.assertIsNone(parse_video_fragment(wrong_count))

    def test_encoder_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            encode_video_fragment(-1, 0, b"x")

        with self.assertRaises(ValueError):
            encode_video_fragment(UINT32_MAX + 1, 0, b"x")

        with self.assertRaises(ValueError):
            encode_video_fragment(1, 1, b"x")

        with self.assertRaises(ValueError):
            fragment_count_for_size(0)

        with self.assertRaises(ValueError):
            fragment_count_for_size(VIDEO_FRAME_MAX_SIZE + 1)


if __name__ == "__main__":
    unittest.main()
