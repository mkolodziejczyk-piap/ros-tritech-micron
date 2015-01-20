# -*- coding: utf-8 -*-

"""Tritech Micron Sonar replies."""

__author__ = "Erin Havens, Jey Kumar, Anass Al-Wohoush"
__version__ = "0.2.0"


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

        # We parse msg header. We check for line terminator.
        self.bitstream.bytepos = 0
        if self.bitstream.endswith('0x0a'):
            self.header = self.bitstream.read('hex:8')
        else:
            raise PacketIncomplete

        # We check for message header '@'.
        if not self.header == '0x40':
            raise PacketCorrupted

        # We find package Hex Length from byte 6, not incl. LF
        # as it is noted in packet bytes 2-5.
        self.bitstream.bytepos = 1
        self.size = self.bitstream.read['uint:32']

        # We check if the size of the packet is correct,
        # by comparing packet's real size to self.size.
        real_size = self.bitstream.len - 56  # 6 bytes & LF (8 bytes)
        if real_size <= self.size:
            raise PacketIncomplete
        elif real_size >= self.size:
            raise PacketCorrupted

        # We check if Bin Length equals Hex Length.
        # Note we read num as little-endian unsigned int.
        self.bitstream.bytepos = 5
        bin_ln = self.bitstream.read('uintle:n')
        if not bin_ln == self.size:
            raise PacketCorrupted

        # We check Packet Source Identification Node 0-240.
        self.bitstream.bytepos = 7
        source_id = self.bitstream.read('uint:8')
        if not source_id >= 0 and source_id <= 240:
            raise PacketCorrupted

        # We check Packet Destination Identification Node 0-240.
        self.bitstream.bytepos = 8
        dest_id = self.bitstream.read('uint:8')
        if not dest_id >= 0 and dest_id <= 240:
            raise PacketCorrupted

        # We check for and parse msg ID 0-72.
        self.bitstream.bytepos = 10
        self.id = self.bitstream.read('uint:8')
        if not self.id >= 0 and self.id <= 72:
            raise PacketCorrupted

        # We parse msg sequence bitset.
        self.bitstream.bytepos = 11
        self.sequence = self.bitstream.read('hex:8')

        # We read bitset to determine number of packets.
        # Necessary for Multi-packet mode.
        pass

        # We check Tx Node number. Should be equal to
        # Packet Source Identification number.
        self.bitstream.bytepos = 12
        tx_node = self.bitstream.read('uint:8')
        if not tx_node == source_id:
            raise PacketCorrupted

        # We parse msg payload. (Byte 14 to end, non LF)
        self.bitstream.bytepos = 13
        size_payload = (self.size - 8) * 8
        self.payload = self.bitstream.read('hex:i', i=size_payload)
