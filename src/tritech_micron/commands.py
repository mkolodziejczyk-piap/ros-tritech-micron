# -*- coding: utf-8 -*-
"""Tritech Micron commands."""

import bitstring

__author__ = "Anass Al-Wohoush, Erin Havens"


class Command(object):

    """Sonar command."""

    def __init__(self, id, payload=None):
        """Constructs Command object.

        Args:
            id: Message ID.
            payload: Message payload (optional).
        """
        self.id = id
        self.payload = payload if payload else bitstring.BitStream()
        self.size = int(self.payload.length / 8) + 8

        # print(f"self.payload:{self.payload}")
        # print(f"self.size:{self.size}")

    def serialize(self):
        """Constructs corresponding string of bytes to send to sonar.

        Returns:
            String representation of data.
        """

        hex_size = bytearray("{:04X}".format(self.size).encode())
        # hex_size = bytearray.fromhex("{:04X}".format(self.size))
        # hex_size = "0x0000" + "{:04X}".format(self.size)
        # hex_size = bitstring.BitArray(32)
        # hex_size.int = self.size
        values = {
            "id": self.id,
            "hex": hex_size,
            "bin": self.size,
            "bytes_left": self.size - 5,
            "payload_length": self.payload.length,
            "payload": self.payload
        }

        serial_format = (
            "0x40, bits:32=hex, uintle:16=bin, 0xFF, 0x02, uint:8=bytes_left,"
            "uint:8=id, 0x80, 0x02, bits:payload_length=payload, 0x0A")
        message = bitstring.pack(serial_format, **values)
        # print(f"message: {message}")
        # print(f"message.tobytes(): {message.tobytes()}")
        return message.tobytes()

        # bitstring.CreationError: Token with length 32 packed with value of length 16 (bits:32=bytearray(b'\x00\x08')).

