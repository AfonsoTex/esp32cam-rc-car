import unittest

from control_protocol import (
    CONTROL_PORT,
    MAX_CONTROL_PACKET_SIZE,
    UINT32_MAX,
    HELLO_PACKET,
    PROTOCOL_VERSION,
    resolve_esp32_address,
    encode_control_command,
)


class ControlProtocolTests(unittest.TestCase):
    def test_hello_contains_current_protocol_version(self):
        self.assertEqual(PROTOCOL_VERSION, 1)
        self.assertEqual(HELLO_PACKET, b"HELLO:1")
        self.assertEqual(CONTROL_PORT, 1883)

    def test_valid_hello_returns_sender_address(self):
        sender = ("192.168.1.74", 54321)

        result = resolve_esp32_address(b"HELLO:1", sender)

        self.assertEqual(result, ("192.168.1.74", 1883))

    def test_invalid_message_is_rejected(self):
        sender = ("192.168.1.74", 1883)

        self.assertIsNone(resolve_esp32_address(b"MOV:0.50", sender))
        self.assertIsNone(resolve_esp32_address(b"HELLO:2", sender))
        self.assertIsNone(resolve_esp32_address(b"", sender))


    def test_control_command_has_canonical_format(self):
        payload = encode_control_command(305419896, 42, 0.5, -0.25)

        self.assertEqual(payload, b"CMD:1,305419896,42,0.50,-0.25")

    def test_control_command_accepts_numeric_boundaries(self):
        payload = encode_control_command(UINT32_MAX, UINT32_MAX, -1.0, 1.0)

        self.assertEqual(
            payload,
            b"CMD:1,4294967295,4294967295,-1.00,1.00",
        )
        self.assertLessEqual(len(payload), MAX_CONTROL_PACKET_SIZE)

    def test_control_command_rejects_invalid_session_and_sequence(self):
        invalid_sessions = (0, -1, UINT32_MAX + 1, True)
        invalid_sequences = (-1, UINT32_MAX + 1, True)

        for session in invalid_sessions:
            with self.subTest(session=session):
                with self.assertRaises(ValueError):
                    encode_control_command(session, 0, 0.0, 0.0)

        for sequence in invalid_sequences:
            with self.subTest(sequence=sequence):
                with self.assertRaises(ValueError):
                    encode_control_command(1, sequence, 0.0, 0.0)

    def test_control_command_rejects_invalid_axes(self):
        invalid_values = (-1.01, 1.01, float("nan"), float("inf"), "0.5")

        for value in invalid_values:
            with self.subTest(move=value):
                with self.assertRaises(ValueError):
                    encode_control_command(1, 0, value, 0.0)

            with self.subTest(direction=value):
                with self.assertRaises(ValueError):
                    encode_control_command(1, 0, 0.0, value)



if __name__ == "__main__":
    unittest.main()
