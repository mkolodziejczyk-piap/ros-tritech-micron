# -*- coding: utf-8 -*-

"""Tritech Micron Sonar exceptions."""

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


class PacketIncomplete(Exception):
    """Packet is incomplete."""
    pass


class PacketCorrupted(Exception):
    """Packet is corrupt."""
    pass
