#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../firmware/main/control_protocol.h"

ControlParseResult parse_text(const char *text, ControlCommand &command) {
    return parse_control_command(
        reinterpret_cast<const uint8_t *>(text),
        strlen(text),
        command
    );
}

void expect_invalid(const char *text) {
    ControlCommand command{};
    assert(parse_text(text, command) != ControlParseResult::OK);
}

int main() {
    ControlCommand command{};

    assert(
        parse_text(
            "CMD:1,305419896,42,0.50,-0.25",
            command
        ) == ControlParseResult::OK
    );
    assert(command.session == 305419896U);
    assert(command.sequence == 42U);
    assert(command.move == 0.50F);
    assert(command.direction == -0.25F);

    assert(
        parse_text(
            "CMD:1,4294967295,4294967295,-1.00,1.00",
            command
        ) == ControlParseResult::OK
    );
    assert(command.session == UINT32_MAX);
    assert(command.sequence == UINT32_MAX);
    assert(command.move == -1.00F);
    assert(command.direction == 1.00F);

    const char *invalid_packets[] = {
        "",
        "HELLO:1",
        "CMD:2,1,0,0.00,0.00",
        "CMD:1,0,0,0.00,0.00",
        "CMD:1,-1,0,0.00,0.00",
        "CMD:1,4294967296,0,0.00,0.00",
        "CMD:1,1,-1,0.00,0.00",
        "CMD:1,1,4294967296,0.00,0.00",
        "CMD:1,1,0,1.01,0.00",
        "CMD:1,1,0,-1.01,0.00",
        "CMD:1,1,0,nan,0.00",
        "CMD:1,1,0,inf,0.00",
        "CMD:1,1,0,+0.50,0.00",
        "CMD:1,1,0,0.5,0.00",
        "CMD:1,1,0,0.500,0.00",
        "CMD:1,1,0,0.00,0.00,EXTRA",
        "CMD:1,1,0,0.00,0.00\n",
        "CMD:1,1,0,0.00",
    };

    for (const char *packet : invalid_packets) {
        expect_invalid(packet);
    }

    uint8_t embedded_nul[] = {
        'C', 'M', 'D', ':', '1', ',', '1', '\0',
        ',', '0', ',', '0', '.', '0', '0',
        ',', '0', '.', '0', '0'
    };
    assert(
        parse_control_command(
            embedded_nul,
            sizeof(embedded_nul),
            command
        ) == ControlParseResult::EMBEDDED_NUL
    );

    uint8_t oversized[CONTROL_PACKET_MAX_SIZE + 1];
    memset(oversized, 'A', sizeof(oversized));
    assert(
        parse_control_command(
            oversized,
            sizeof(oversized),
            command
        ) == ControlParseResult::PACKET_TOO_LARGE
    );

    ControlSequenceState state{};
    ControlCommand first{100U, 10U, 0.0F, 0.0F};

    assert(accept_control_sequence(first, state));
    assert(!accept_control_sequence(first, state));

    ControlCommand older{100U, 9U, 0.0F, 0.0F};
    assert(!accept_control_sequence(older, state));

    ControlCommand newer{100U, 11U, 0.0F, 0.0F};
    assert(accept_control_sequence(newer, state));

    ControlCommand new_session{200U, 0U, 0.0F, 0.0F};
    assert(accept_control_sequence(new_session, state));
    assert(!accept_control_sequence(new_session, state));

    ControlCommand delayed_old_session{100U, 12U, 0.0F, 0.0F};
    assert(!accept_control_sequence(delayed_old_session, state));

    ControlCommand third_session{300U, 5U, 0.0F, 0.0F};
    assert(accept_control_sequence(third_session, state));

    ControlCommand delayed_second_session{200U, 1U, 0.0F, 0.0F};
    assert(!accept_control_sequence(delayed_second_session, state));

    puts("Firmware control protocol tests passed.");
    return 0;
}
