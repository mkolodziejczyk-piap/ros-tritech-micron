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


class SonarNotConfigured(Exception):
    """Sonar is not configured for scanning."""
    pass


class SonarNotFound(Exception):
    """Sonar port could not be found."""
    pass


class SonarNotInitialized(Exception):
    """Sonar not initialized."""
    pass


class TimeoutError(Exception):
    """Communication timed out."""
    pass
