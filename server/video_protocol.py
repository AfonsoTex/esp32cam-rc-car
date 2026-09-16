"""Framing protocol for JPEG video sent over UDP."""

import math
import struct
import time

VIDEO_MAGIC = b"VFR1"
VIDEO_HEADER = struct.Struct("!4sIHHI")

VIDEO_HEADER_SIZE = VIDEO_HEADER.size
VIDEO_DATAGRAM_MAX_SIZE = 1200
VIDEO_FRAGMENT_PAYLOAD_SIZE = VIDEO_DATAGRAM_MAX_SIZE - VIDEO_HEADER_SIZE
VIDEO_FRAME_MAX_SIZE = 512 * 1024
UINT32_MAX = (1 << 32) - 1


def fragment_count_for_size(frame_size):
    if not isinstance(frame_size, int):
        raise TypeError("frame_size must be an integer")

    if frame_size <= 0 or frame_size > VIDEO_FRAME_MAX_SIZE:
        raise ValueError("frame_size is outside the allowed range")

    return math.ceil(frame_size / VIDEO_FRAGMENT_PAYLOAD_SIZE)


def encode_video_fragment(frame_id, fragment_index, frame):
    if not isinstance(frame_id, int) or not 0 <= frame_id <= UINT32_MAX:
        raise ValueError("frame_id must be an unsigned 32-bit integer")

    if not isinstance(frame, bytes):
        raise TypeError("frame must be bytes")

    fragment_count = fragment_count_for_size(len(frame))

    if not isinstance(fragment_index, int):
        raise TypeError("fragment_index must be an integer")

    if not 0 <= fragment_index < fragment_count:
        raise ValueError("fragment_index is outside the frame")

    offset = fragment_index * VIDEO_FRAGMENT_PAYLOAD_SIZE
    payload = frame[offset:offset + VIDEO_FRAGMENT_PAYLOAD_SIZE]

    header = VIDEO_HEADER.pack(
        VIDEO_MAGIC,
        frame_id,
        fragment_index,
        fragment_count,
        len(frame),
    )

    return header + payload


def parse_video_fragment(datagram):
    if not isinstance(datagram, bytes):
        return None

    if not VIDEO_HEADER_SIZE < len(datagram) <= VIDEO_DATAGRAM_MAX_SIZE:
        return None

    magic, frame_id, fragment_index, fragment_count, frame_size = (
        VIDEO_HEADER.unpack_from(datagram)
    )

    if magic != VIDEO_MAGIC:
        return None

    if frame_size <= 0 or frame_size > VIDEO_FRAME_MAX_SIZE:
        return None

    expected_count = math.ceil(
        frame_size / VIDEO_FRAGMENT_PAYLOAD_SIZE
    )

    if fragment_count != expected_count:
        return None

    if fragment_index >= fragment_count:
        return None

    payload = datagram[VIDEO_HEADER_SIZE:]
    offset = fragment_index * VIDEO_FRAGMENT_PAYLOAD_SIZE
    expected_payload_size = min(
        VIDEO_FRAGMENT_PAYLOAD_SIZE,
        frame_size - offset,
    )

    if len(payload) != expected_payload_size:
        return None

    return (
        frame_id,
        fragment_index,
        fragment_count,
        frame_size,
        payload,
    )


class VideoFrameAssembler:
    """Reassemble only the newest valid JPEG frame."""

    def __init__(self, timeout_seconds=0.5):
        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")

        self.timeout_seconds = timeout_seconds
        self.latest_frame_id = None
        self.active_frame_id = None
        self.fragment_count = 0
        self.frame_size = 0
        self.fragments = {}
        self.last_fragment_time = 0.0

    @staticmethod
    def _is_newer(candidate, reference):
        difference = (candidate - reference) & UINT32_MAX
        return 0 < difference < (1 << 31)

    def _clear_active_frame(self):
        self.active_frame_id = None
        self.fragment_count = 0
        self.frame_size = 0
        self.fragments = {}
        self.last_fragment_time = 0.0

    def _start_frame(
        self,
        frame_id,
        fragment_count,
        frame_size,
        now,
    ):
        self.latest_frame_id = frame_id
        self.active_frame_id = frame_id
        self.fragment_count = fragment_count
        self.frame_size = frame_size
        self.fragments = {}
        self.last_fragment_time = now

    def add_datagram(self, datagram, now=None):
        parsed = parse_video_fragment(datagram)

        if parsed is None:
            return None

        if now is None:
            now = time.monotonic()

        if (
            self.active_frame_id is not None
            and now - self.last_fragment_time > self.timeout_seconds
        ):
            self._clear_active_frame()

        (
            frame_id,
            fragment_index,
            fragment_count,
            frame_size,
            payload,
        ) = parsed

        if self.active_frame_id is None:
            if (
                self.latest_frame_id is not None
                and not self._is_newer(
                    frame_id,
                    self.latest_frame_id,
                )
            ):
                return None

            self._start_frame(
                frame_id,
                fragment_count,
                frame_size,
                now,
            )

        elif frame_id != self.active_frame_id:
            if not self._is_newer(frame_id, self.latest_frame_id):
                return None

            self._clear_active_frame()
            self._start_frame(
                frame_id,
                fragment_count,
                frame_size,
                now,
            )

        elif (
            fragment_count != self.fragment_count
            or frame_size != self.frame_size
        ):
            self._clear_active_frame()
            return None

        previous = self.fragments.get(fragment_index)

        if previous is not None:
            if previous != payload:
                self._clear_active_frame()

            return None

        self.fragments[fragment_index] = payload
        self.last_fragment_time = now

        if len(self.fragments) != self.fragment_count:
            return None

        frame = b"".join(
            self.fragments[index]
            for index in range(self.fragment_count)
        )
        completed_frame_id = self.active_frame_id
        expected_size = self.frame_size
        self._clear_active_frame()

        if len(frame) != expected_size:
            return None

        return completed_frame_id, frame
