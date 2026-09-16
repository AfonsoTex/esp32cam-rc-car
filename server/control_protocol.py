"""Protocol definitions for the UDP control channel."""

PROTOCOL_VERSION = 1
CONTROL_PORT = 1883
HELLO_PACKET = f"HELLO:{PROTOCOL_VERSION}".encode("ascii")


def resolve_esp32_address(payload: bytes, sender: tuple[str, int]):
    """Return the ESP32 UDP address when a valid HELLO is received."""
    if payload != HELLO_PACKET:
        return None

    return sender[0], CONTROL_PORT
