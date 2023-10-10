# -*- coding: utf-8 -*-
"""Tritech Micron serial communication handler."""

import errno
import rclpy
import serial
import select
import bitstring
from tritech_micron.replies import Reply
from tritech_micron.messages import Message
from tritech_micron.commands import Command
from tritech_micron.exceptions import PacketIncomplete

__author__ = "Anass Al-Wohoush, Jana Pavlasek, Malcolm Watt"


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
        # rospy.logdebug("Sending %s: %s", Message.to_string(message), payload)
        self.conn.write(cmd.serialize())

    def get_reply(self):
        """Waits for and returns Reply.

        Returns:
            First complete reply if expected message ID was not specified,
            otherwise first complete reply of expected message ID.

        Raises:
            PacketCorrupted: Packet is corrupt.
        """
        # try:
        # Wait for the '@' character.
        # char_read = self.conn.read()
        while not self.conn.read() == b'@':
        # while not char_read == "@":
            # rospy.loginfo("while 1")
            # char_read = self.conn.read()
            # print(char_read)
            pass

        # Read one line at a time until packet is complete and parsed.
        packet = bitstring.BitStream("0x40")
        while True:
            # rospy.loginfo("while 2")
            # Read until new line.
            current_line = self.conn.readline()
            # print(current_line)
            # print(current_line.decode())
            for char in current_line:
                # print(char)
                # packet.append("0x{:02X}".format(ord(char.decode())))
                packet.append("0x{:02X}".format(char))

            # Try to parse.
            try:
                reply = Reply(packet)
                break
            except PacketIncomplete:
                # Keep looking.
                continue

        # rospy.logdebug("Received %s: %s", reply.name, reply.payload)
        return reply
        # except select.error as (code, msg):
        #     # Set SIGINT as KeyboardInterrupt correctly, because pyserial has
        #     # problems.
        #     if code == errno.EINTR:
        #         raise KeyboardInterrupt()

        #     # Otherwise, reraise.
        #     raise
