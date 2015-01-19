# -*- coding: utf-8 -*-

"""Tritech Micron Sonar serial communication handler."""

__author__ = "Anass Al-Wohoush, Jana Pavlasek, Malcolm Watt"
__version__ = "0.0.2"


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

        # Waits for the '@' char.
        while True:
            current_char = self.conn.read(1)
            if current_char == '@':
                bitstream.append(current_char)
                break

        # Parses minimum packet length by default.
        bitstream.append(self.conn.read(13))

        # Keep reading one byte at a time until packet's complete and parsed.
        while True:
            # If byte is end of packet, try to parse.
            current_char = self.conn.read(1)
            bitstream.append(current_char)

            if current_char == 0x0A:
                try:
                    reply = Reply(bitstream)
                    break
                except PacketIncomplete:
                    # Keep looking.
                    continue

        return reply
