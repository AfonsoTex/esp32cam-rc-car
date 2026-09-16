import unittest

from control_protocol import (
    CONTROL_PORT,
    HELLO_PACKET,
    PROTOCOL_VERSION,
    resolve_esp32_address,
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


if __name__ == "__main__":
    unittest.main()
