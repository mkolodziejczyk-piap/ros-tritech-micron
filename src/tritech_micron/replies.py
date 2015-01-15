# -*- coding: utf-8 -*-

"""Tritech Micron Sonar replies."""

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


import bitstring
from exceptions import PacketIncomplete, PacketCorrupted


class Reply(object):

    """Parses and verifies reply packets.

    Attributes:
        header: 13 byte message header.
        id: Message ID.
        size: Number of bytes in packet from 6th byte onwards.
        payload: Data excluding header and end character.
    """

    def __init__(self, bitstream):
        """Constructs Reply object.

        Args:
            bitstream: BitStream holding packet to parse.

        Raises:
            PacketIncomplete: Packet is incomplete.
            PacketCorrupted: Packet is corrupted.
        """
        self.bitstream = bitstream

        self.header = None
        self.id = None
        self.size = None
        self.payload = None

        self.parse()

    def parse(self):
        """Parses packet into header, message ID, sequence and payload.

        This method also checks if the packet contains all relevant fields and
        has the correct size.

        Raises:
            PacketIncomplete: Packet is incomplete.
            PacketCorrupted: Packet is corrupted.
        """
        pass
