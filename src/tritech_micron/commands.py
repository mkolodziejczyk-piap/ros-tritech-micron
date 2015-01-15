# -*- coding: utf-8 -*-

"""Tritech Micron Sonar commands."""

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


class Command(object):

    """Enumeration of available commands and their respective message IDs."""

    # TODO: Complete list.
    REBOOT = 16  # Reboots device.
    HEAD_COMMAND = 19  # Update parameters.
    SEND_VERSION = 23  # Request version.
