# -*- coding: utf-8 -*-

"""Tritech Micron Sonar serial communication handler."""

import serial
import bitstring
from replies import Reply
from commands import Command
from exceptions import PacketIncomplete

__author__ = "Anass Al-Wohoush, Jana Pavlasek, Malcolm Watt"
__version__ = "0.0.3"


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
        self.conn = serial.Serial(port=port, baudrate=115200)
        self._queue =

    def open(self):
        """Opens serial connection."""
        self.conn.open()

    def close(self):
        """Closes serial connection."""
        self.conn.close()

    def send(self, message, payload=None):
        """Formats message and payload into packet and sends it to device.

        Args:
            command: Command to send.
            payload: Additional payload to send in packet.
        """
        cmd = Command(message, payload)
        self.conn.write(cmd.to_string())

    def get_reply(self, expected=None):
        """Waits for and returns Reply.

        Returns:
            First complete reply if no message ID was expected, otherwise
            first complete reply of expected message ID.

        Raises:
            PacketCorrupted: Packet is corrupt.
        """
        done = False
        while not done:
            bitstream = bitstring.BitStream("0x40")

            # Waits for the '@' char.
            self.conn.readline(eol="@")

            # Read one line at a time until packet is complete and parsed.
            while True:
                # Read until new line.
                current_line = self.conn.readline()
                bitstream.append(current_line)

                # Try to parse.
                try:
                    reply = Reply(bitstream)
                    break
                except PacketIncomplete:
                    # Keep looking.
                    continue

            # Verify packet received is the one expected.
            if expected is not None and reply.id == expected:
                break

        return reply
