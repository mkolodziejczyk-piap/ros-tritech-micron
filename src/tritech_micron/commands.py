# -*- coding: utf-8 -*-

"""Tritech Micron Sonar commands."""

import struct
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

    def to_string(self):
        """Constructs corresponding string of bytes to send to sonar.

        Returns:
            String representation of data.
        """
        header = "@"
        tx_node = chr(0xFF)
        rx_node = chr(0x02)
        message_id = chr(self.id)
        sequence = chr(0x80)
        node = chr(0x02)
        line_feed = chr(0x0A)

        _size = (self.payload.length + 8) / 8
        hex_size = "{:0>4}".format(hex(_size)[2:])
        bin_size = struct.pack("<h", _size)
        bytes_left = chr(_size - 5)

        return "".join((
            header, hex_size, bin_size, tx_node, rx_node, bytes_left,
            message_id, self.payload.tobytes(), sequence, node, line_feed
        ))
