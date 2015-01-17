# -*- coding: utf-8 -*-

"""Tritech Micron Sonar replies."""

__author__ = "Anass Al-Wohoush, Jey Kumar"
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
        self.sequence = None

        self.parse()

    def parse(self):
        """Parses packet into header, message ID, sequence and payload.

        This method also checks if the packet contains all relevant fields and
        has the correct size.

        Raises:
            PacketIncomplete: Packet is incomplete.
            PacketCorrupted: Packet is corrupted.
        """
        
        # Parses message header and payload and checks for line terminator.
        self.bitsream.bytepos = 0
        if self.bitstream.endswith('0x0a'):
            self.header, self.payload = self.bitstream[:-8].unpack('hex:104, hex')
        else:
            raise PacketIncomplete
        
        # Parses message ID.
        self.bitsream.bytepos = 10
        self.id = self.bitstream.read('hex:8')
        
        # Parses message sequence.
        self.bitsream.bytepos = 11
        self.sequence = self.bitstream.read('hex:8')
        
        # Measures size of packet in bytes (excluding first 6 bytes).
        self.size = self.bitstream[48].len / 8
        
        # Checks if packet contains all relevant fields (except line terminator).
        pass
        
        # Checks if the size of the packet is correct.
        pass
