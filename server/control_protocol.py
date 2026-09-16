"""Protocol definitions for the UDP control channel."""

import math
from numbers import Real

PROTOCOL_VERSION = 1
CONTROL_PORT = 1883
MAX_CONTROL_PACKET_SIZE = 64
UINT32_MAX = (1 << 32) - 1
HELLO_PACKET = f"HELLO:{PROTOCOL_VERSION}".encode("ascii")


def resolve_esp32_address(payload: bytes, sender: tuple[str, int]):
    """Return the ESP32 UDP address when a valid HELLO is received."""
    if payload != HELLO_PACKET:
        return None

    return sender[0], CONTROL_PORT


def _validate_uint32(name: str, value: int, *, allow_zero: bool) -> None:
    minimum = 0 if allow_zero else 1

    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")

    if not minimum <= value <= UINT32_MAX:
        raise ValueError(f"{name} must be between {minimum} and {UINT32_MAX}")


def _validate_axis(name: str, value: Real) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a real number")

    numeric_value = float(value)
    if not math.isfinite(numeric_value) or not -1.0 <= numeric_value <= 1.0:
        raise ValueError(f"{name} must be finite and between -1.0 and 1.0")

    return numeric_value


def encode_control_command(
    session: int,
    sequence: int,
    move: Real,
    direction: Real,
) -> bytes:
    """Encode one complete motor state as a versioned UDP datagram."""
    _validate_uint32("session", session, allow_zero=False)
    _validate_uint32("sequence", sequence, allow_zero=True)

    move_value = _validate_axis("move", move)
    direction_value = _validate_axis("direction", direction)

    payload = (
        f"CMD:{PROTOCOL_VERSION},{session},{sequence},"
        f"{move_value:.2f},{direction_value:.2f}"
    ).encode("ascii")

    if len(payload) > MAX_CONTROL_PACKET_SIZE:
        raise ValueError("encoded control command is too large")

    return payload
