# -*- coding: utf-8 -*-

"""Tritech Micron Sonar replies."""

__author__ = "Erin Havens, Jey Kumar, Anass Al-Wohoush"
__version__ = "0.4.0"


from exceptions import PacketIncomplete, PacketCorrupted


class Reply(object):

    """Parses and verifies reply packets.

    Attributes:
        id: Message ID.
        is_last: Whether packet is last in sequence.
        payload: Data excluding header and end character.
        sequence: Packet sequence.
        size: Number of bytes in packet from 6th byte onwards.
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

        self.id = None
        self.is_last = None
        self.payload = None
        self.sequence = None
        self.size = None

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
            header = self.bitstream.read('uint:8')
        else:
            raise PacketIncomplete

        # We check for message header '@'.
        if header != 0x40:
            raise PacketCorrupted

        # We find package Hex Length from byte 6, excluding LF
        # as it is noted in packet bytes 2-5.
        self.bitstream.bytepos = 1
        hex_list = [self.bitstream.read('hex:8') for i in range(4)]
        ascii_string = ''.join([chr(i)for i in hex_list])
        self.size = int(ascii_string, 16)

        # We check if the size of the packet is correct,
        # by comparing packet's real size to self.size.
        real_size = (self.bitstream.len / 8) - 6  # 6 bytes
        if real_size <= self.size:
            raise PacketIncomplete
        elif real_size >= self.size:
            raise PacketCorrupted

        # We check if Bin Length equals Hex Length.
        # Note we read num as little-endian unsigned int.
        self.bitstream.bytepos = 5
        bin_ln = self.bitstream.read('uintle:16')
        if bin_ln != self.size:
            raise PacketCorrupted

        # We parse Packet Source Identification Node.
        self.bitstream.bytepos = 7
        source_id = self.bitstream.read('uint:8')

        # We check Packet Destination Identification Node is 255.
        self.bitstream.bytepos = 8
        dest_id = self.bitstream.read('uint:8')
        if dest_id != 255:
            raise PacketCorrupted

        # We check for size following byte 10, excluding LF.
        self.bitstream.bytepos = 9
        byte_count = self.bitstream.read('uint:8')
        if self.id == 2 and byte_count == 0:
            # mtHeadData single-packet replies are different; always 0.
            # Could be used to confirm whether it's in single-packet mode.
            pass
        else:
            # For all other replies, check size.
            # Byte count differs from self.size by 5 bytes.
            if byte_count != self.size - 5:
                raise PacketCorrupted

        # We check for and parse msg ID 0-72.
        self.bitstream.bytepos = 10
        self.id = self.bitstream.read('uint:8')
        if not 0 <= self.id <= 72:
            raise PacketCorrupted

        # We parse msg sequence bitset.
        self.bitstream.bytepos = 11
        self.is_last = self.bitstream.read('bool')
        self.sequence = self.bitstream.read('uint:7')

        # We read bitset to determine number of packets.
        # Necessary for Multi-packet mode.
        pass

        # We check Tx Node number. Should be equal to
        # Packet Source Identification number.
        self.bitstream.bytepos = 12
        tx_node = self.bitstream.read('uint:8')
        if tx_node != source_id:
            raise PacketCorrupted

        # We parse msg payload (byte 14 to end, excluding LF).
        self.bitstream.bytepos = 13
        size_payload = self.size * 8
        self.payload = self.bitstream.read('hex:i', i=size_payload)
