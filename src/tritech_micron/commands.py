# -*- coding: utf-8 -*-

"""Tritech Micron Sonar commands."""

__author__ = "Erin Havens, Anass Al-Wohoush"
__version__ = "0.1.0"


class Command(object):

    """Enumeration of available commands and their respective message IDs."""

    NULL = 0
    VERSION_DATA = 1
    HEAD_DATA = 2  # The MSG reply in response to SEND_DATA commands.
    SPECT_DATA = 3
    ALIVE = 4  # On power-up, the broadcast state of Motor, Transducer, Params.
    PRG_ACK = 5
    BB_USER_DATA = 6
    TEST_DATA = 7
    AUD_DATA = 8
    ADC_DATA = 9
    ADC_REQ = 10
    LAN_STATUS = 13
    SET_TIME = 14
    TIMEOUT = 15
    REBOOT = 16  # Reboots device.
    PERFORMANCE_DATA = 17
    HEAD_COMMAND = 19  # Update parameters.
    ERASE_SECTOR = 20
    PROG_BLOCK = 21
    COPY_BOOT_BLK = 22
    SEND_VERSION = 23  # Request version.
    SEND_BB_USER = 24
    SEND_DATA = 25  # Perform sample set.
    SED_PERF_NCE_DATA = 26
    FPGA_TEST = 40
    FPGA_ERASE = 41
    FPGA_PROGRAM = 42
    SEND_FPGA_FLASH_ST = 47
    FPGA_TEST_DATA = 48
    FPGA_FLASH_ST_DATA = 49
    SEND_FPGA_VERSION = 56
    FPGA_VERSION_DATA = 57
    FGPA_DO_CALIBRATE = 61
    SEND_FPGA_CAL_DATA = 62
    FPGA_CAL_DATA = 63
    ZERO_FPGA_CAL = 64
    STOP_ALIVES = 66
    RESET_TO_DEFAULTS = 70
    CHANGE_VER_DATA = 71
    FGPA_PROG_USR_CDE = 72

