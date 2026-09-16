#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

constexpr uint32_t CONTROL_PROTOCOL_VERSION = 1;
constexpr size_t CONTROL_PACKET_MAX_SIZE = 64;
constexpr size_t CONTROL_RETIRED_SESSION_CAPACITY = 4;

struct ControlCommand {
    uint32_t session;
    uint32_t sequence;
    float move;
    float direction;
};

enum class ControlParseResult : uint8_t {
    OK,
    EMPTY_PACKET,
    PACKET_TOO_LARGE,
    EMBEDDED_NUL,
    BAD_PREFIX,
    INVALID_VERSION,
    INVALID_SESSION,
    INVALID_SEQUENCE,
    INVALID_MOVE,
    INVALID_DIRECTION,
};

struct ControlSequenceState {
    bool initialized = false;
    uint32_t session = 0;
    uint32_t last_sequence = 0;
    uint32_t retired_sessions[CONTROL_RETIRED_SESSION_CAPACITY]{};
    size_t retired_count = 0;
    size_t retired_next = 0;
};

namespace control_protocol_detail {

inline bool parse_uint32_field(
    const char *&cursor,
    char delimiter,
    uint32_t &output,
    bool allow_zero
) {
    if (*cursor < '0' || *cursor > '9') {
        return false;
    }

    uint32_t value = 0;

    while (*cursor >= '0' && *cursor <= '9') {
        const uint32_t digit = static_cast<uint32_t>(*cursor - '0');

        if (value > (UINT32_MAX - digit) / 10U) {
            return false;
        }

        value = value * 10U + digit;
        ++cursor;
    }

    if (*cursor != delimiter || (!allow_zero && value == 0)) {
        return false;
    }

    if (delimiter != '\0') {
        ++cursor;
    }

    output = value;
    return true;
}

inline bool parse_axis_field(
    const char *&cursor,
    char delimiter,
    float &output
) {
    bool negative = false;

    if (*cursor == '-') {
        negative = true;
        ++cursor;
    }

    if (*cursor != '0' && *cursor != '1') {
        return false;
    }

    const uint8_t whole = static_cast<uint8_t>(*cursor - '0');
    ++cursor;

    if (*cursor != '.') {
        return false;
    }
    ++cursor;

    if (
        cursor[0] < '0' || cursor[0] > '9'
        || cursor[1] < '0' || cursor[1] > '9'
    ) {
        return false;
    }

    const uint8_t decimal =
        static_cast<uint8_t>((cursor[0] - '0') * 10 + (cursor[1] - '0'));
    cursor += 2;

    if (*cursor != delimiter) {
        return false;
    }

    if (whole == 1 && decimal != 0) {
        return false;
    }

    if (delimiter != '\0') {
        ++cursor;
    }

    float value = static_cast<float>(whole)
        + static_cast<float>(decimal) / 100.0F;

    output = negative ? -value : value;
    return true;
}

}  // namespace control_protocol_detail

inline ControlParseResult parse_control_command(
    const uint8_t *payload,
    size_t length,
    ControlCommand &output
) {
    if (payload == nullptr || length == 0) {
        return ControlParseResult::EMPTY_PACKET;
    }

    if (length > CONTROL_PACKET_MAX_SIZE) {
        return ControlParseResult::PACKET_TOO_LARGE;
    }

    for (size_t index = 0; index < length; ++index) {
        if (payload[index] == '\0') {
            return ControlParseResult::EMBEDDED_NUL;
        }
    }

    char text[CONTROL_PACKET_MAX_SIZE + 1];
    memcpy(text, payload, length);
    text[length] = '\0';

    constexpr char prefix[] = "CMD:";
    constexpr size_t prefix_length = sizeof(prefix) - 1;

    if (
        length <= prefix_length
        || memcmp(text, prefix, prefix_length) != 0
    ) {
        return ControlParseResult::BAD_PREFIX;
    }

    const char *cursor = text + prefix_length;
    ControlCommand parsed{};
    uint32_t version = 0;

    using control_protocol_detail::parse_axis_field;
    using control_protocol_detail::parse_uint32_field;

    if (!parse_uint32_field(cursor, ',', version, true)) {
        return ControlParseResult::INVALID_VERSION;
    }

    if (version != CONTROL_PROTOCOL_VERSION) {
        return ControlParseResult::INVALID_VERSION;
    }

    if (!parse_uint32_field(cursor, ',', parsed.session, false)) {
        return ControlParseResult::INVALID_SESSION;
    }

    if (!parse_uint32_field(cursor, ',', parsed.sequence, true)) {
        return ControlParseResult::INVALID_SEQUENCE;
    }

    if (!parse_axis_field(cursor, ',', parsed.move)) {
        return ControlParseResult::INVALID_MOVE;
    }

    if (!parse_axis_field(cursor, '\0', parsed.direction)) {
        return ControlParseResult::INVALID_DIRECTION;
    }

    output = parsed;
    return ControlParseResult::OK;
}

inline bool accept_control_sequence(
    const ControlCommand &command,
    ControlSequenceState &state
) {
    if (!state.initialized) {
        state.initialized = true;
        state.session = command.session;
        state.last_sequence = command.sequence;
        return true;
    }

    if (command.session == state.session) {
        if (command.sequence <= state.last_sequence) {
            return false;
        }

        state.last_sequence = command.sequence;
        return true;
    }

    for (size_t index = 0; index < state.retired_count; ++index) {
        if (command.session == state.retired_sessions[index]) {
            return false;
        }
    }

    if (state.retired_count < CONTROL_RETIRED_SESSION_CAPACITY) {
        state.retired_sessions[state.retired_count] = state.session;
        ++state.retired_count;
    } else {
        state.retired_sessions[state.retired_next] = state.session;
        state.retired_next = (
            state.retired_next + 1
        ) % CONTROL_RETIRED_SESSION_CAPACITY;
    }

    state.session = command.session;
    state.last_sequence = command.sequence;
    return true;
}

#endif  // CONTROL_PROTOCOL_H
