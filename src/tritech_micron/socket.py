# -*- coding: utf-8 -*-

"""Tritech Micron Sonar serial communication handler."""

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


import serial
import bitstring
from replies import Reply
from exceptions import PacketIncomplete


class Socket(object):

    """Serial communication socket.

    Attributes:
        conn: Serial connection.
    """

    def __init__(self, port):
        """Constructs Socket object.

        Args:
            port: Serial port.
        """
        self.conn = serial.Serial(port=port)

    def open(self):
        self.conn.open()

    def close(self):
        self.conn.close()

    def send(self, command, payload=None):
        """Formats command and payload into packet and sends it to device.

        Args:
            command: Command to send.
            payload: Additional payload to send in packet.
        """
        pass

    def get_reply(self):
        """Waits for and returns Reply.

        Returns:
            Reply.

        Raises:
            PacketCorrupted: Packet is corrupt.
        """
        bitstream = bitstring.BitStream()

        # Read one byte at a time until 13 bytes are collected from header '@'.
        pass

        # Keep reading one byte at a time until packet's complete and parsed.
        while True:
            # If byte is end of packet, try to parse.
            if False:
                try:
                    reply = Reply(bitstream)
                    break
                except PacketIncomplete:
                    # Keep looking.
                    continue

        return reply
