# -*- coding: utf-8 -*-

"""Tritech Micron Sonar serial communication handler."""

import rospy
import serial
import bitstring
from replies import Reply
from messages import Message
from commands import Command
from exceptions import PacketIncomplete

__author__ = "Anass Al-Wohoush, Jana Pavlasek, Malcolm Watt"
__version__ = "0.6.0"


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
        rospy.loginfo("Sending %s: %s", Message.to_string(message), payload)
        rospy.logdebug(
            "Sending %s",
            "".join(["{:02X}".format(ord(x)) for x in cmd.to_string()])
        )
        self.conn.write(cmd.to_string())

    def get_reply(self):
        """Waits for and returns Reply.

        Returns:
            First complete reply if expected message ID was not specified,
            otherwise first complete reply of expected message ID.

        Raises:
            PacketCorrupted: Packet is corrupt.
        """
        # Wait for the '@' character.
        while not self.conn.read() == "@":
            pass

        # Read one line at a time until packet is complete and parsed.
        packet = bitstring.BitStream("0x40")
        while True:
            # Read until new line.
            current_line = self.conn.readline()
            for char in current_line:
                packet.append("0x{:02X}".format(ord(char)))

            # Try to parse.
            try:
                reply = Reply(packet)
                break
            except PacketIncomplete:
                # Keep looking.
                continue

        rospy.loginfo("Received %s: %s", reply.type, reply.payload)
        return reply
