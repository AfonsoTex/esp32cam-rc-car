#ifndef VIDEO_PROTOCOL_H
#define VIDEO_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

constexpr size_t VIDEO_HEADER_SIZE = 16;
constexpr size_t VIDEO_DATAGRAM_MAX_SIZE = 1200;
constexpr size_t VIDEO_FRAGMENT_PAYLOAD_SIZE =
    VIDEO_DATAGRAM_MAX_SIZE - VIDEO_HEADER_SIZE;
constexpr uint32_t VIDEO_FRAME_MAX_SIZE = 512U * 1024U;

constexpr uint8_t VIDEO_MAGIC[4] = {
    'V',
    'F',
    'R',
    '1',
};

inline uint16_t video_fragment_count(uint32_t frame_size) {
    if (frame_size == 0 || frame_size > VIDEO_FRAME_MAX_SIZE) {
        return 0;
    }

    return static_cast<uint16_t>(
        (frame_size + VIDEO_FRAGMENT_PAYLOAD_SIZE - 1)
        / VIDEO_FRAGMENT_PAYLOAD_SIZE
    );
}

inline void video_write_uint16_be(
    uint8_t *destination,
    uint16_t value
) {
    destination[0] = static_cast<uint8_t>(value >> 8);
    destination[1] = static_cast<uint8_t>(value);
}

inline void video_write_uint32_be(
    uint8_t *destination,
    uint32_t value
) {
    destination[0] = static_cast<uint8_t>(value >> 24);
    destination[1] = static_cast<uint8_t>(value >> 16);
    destination[2] = static_cast<uint8_t>(value >> 8);
    destination[3] = static_cast<uint8_t>(value);
}

inline bool encode_video_header(
    uint8_t *output,
    size_t output_size,
    uint32_t frame_id,
    uint16_t fragment_index,
    uint16_t fragment_count,
    uint32_t frame_size
) {
    if (output == nullptr || output_size < VIDEO_HEADER_SIZE) {
        return false;
    }

    const uint16_t expected_count =
        video_fragment_count(frame_size);

    if (
        expected_count == 0
        || fragment_count != expected_count
        || fragment_index >= fragment_count
    ) {
        return false;
    }

    output[0] = VIDEO_MAGIC[0];
    output[1] = VIDEO_MAGIC[1];
    output[2] = VIDEO_MAGIC[2];
    output[3] = VIDEO_MAGIC[3];

    video_write_uint32_be(output + 4, frame_id);
    video_write_uint16_be(output + 8, fragment_index);
    video_write_uint16_be(output + 10, fragment_count);
    video_write_uint32_be(output + 12, frame_size);

    return true;
}

#endif  // VIDEO_PROTOCOL_H
