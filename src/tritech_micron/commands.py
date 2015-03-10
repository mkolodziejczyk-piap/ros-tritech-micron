# -*- coding: utf-8 -*-

"""Tritech Micron Sonar commands."""

import bitstring

__author__ = "Anass Al-Wohoush, Erin Havens"
__version__ = "0.5.0"


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
        self.size = (self.payload.length / 8) + 8

    def to_string(self):
        """Constructs corresponding string of bytes to send to sonar.

        Returns:
            String representation of data.
        """
        ascii_size = map(lambda x: hex(ord(x)), "{:04X}".format(self.size))
        hex_size = bitstring.pack(", ".join(ascii_size))
        values = {
            "id": self.id,
            "hex": hex_size,
            "bin": self.size,
            "bytes_left": self.size - 5,
            "payload_length": self.payload.length,
            "payload": self.payload
        }

        format = (
            "0x40, bits:32=hex, uintle:16=bin, 0xFF, 0x02, uint:8=bytes_left,"
            "uint:8=id, 0x80, 0x02, bits:payload_length=payload, 0x0A"
        )
        message = bitstring.pack(format, **values)
        return message.tobytes()
