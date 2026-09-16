#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "../firmware/main/video_protocol.h"

int main() {
    static_assert(VIDEO_HEADER_SIZE == 16);
    static_assert(VIDEO_DATAGRAM_MAX_SIZE == 1200);
    static_assert(VIDEO_FRAGMENT_PAYLOAD_SIZE == 1184);

    assert(video_fragment_count(0) == 0);
    assert(video_fragment_count(1) == 1);
    assert(video_fragment_count(1184) == 1);
    assert(video_fragment_count(1185) == 2);
    assert(video_fragment_count(2500) == 3);
    assert(video_fragment_count(VIDEO_FRAME_MAX_SIZE) == 443);
    assert(video_fragment_count(VIDEO_FRAME_MAX_SIZE + 1) == 0);

    std::array<uint8_t, VIDEO_HEADER_SIZE> header{};

    assert(
        encode_video_header(
            header.data(),
            header.size(),
            0x01020304U,
            2,
            3,
            2500
        )
    );

    const std::array<uint8_t, VIDEO_HEADER_SIZE> expected = {
        0x56, 0x46, 0x52, 0x31,
        0x01, 0x02, 0x03, 0x04,
        0x00, 0x02,
        0x00, 0x03,
        0x00, 0x00, 0x09, 0xC4,
    };

    assert(header == expected);

    assert(
        !encode_video_header(
            nullptr,
            VIDEO_HEADER_SIZE,
            1,
            0,
            1,
            1
        )
    );

    assert(
        !encode_video_header(
            header.data(),
            VIDEO_HEADER_SIZE - 1,
            1,
            0,
            1,
            1
        )
    );

    assert(
        !encode_video_header(
            header.data(),
            header.size(),
            1,
            1,
            1,
            1
        )
    );

    assert(
        !encode_video_header(
            header.data(),
            header.size(),
            1,
            0,
            2,
            1
        )
    );

    std::puts("Firmware video protocol tests passed.");
    return 0;
}
