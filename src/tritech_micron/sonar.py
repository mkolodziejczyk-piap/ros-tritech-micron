# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

import struct
import datetime
import bitstring
from socket import Socket
from messages import Message

__author__ = "Anass Al-Wohoush"
__version__ = "0.5.0"


class Sonar(object):

    """Tritech Micron Sonar.

    Attributes:
        adc8on: 8-bit resolution ADC is on.
        conn: Serial connection.
        ...
    """

    def __init__(self, port="/dev/sonar", inverted=False):
        """Constructs Sonar object.

        Args:
            port: Serial port.
        """
        self.initialized = False
        self.inverted = True

        self.adc8on = True
        self.continuous = True
        self.scanright = True
        self.step = 16
        self.ad_low = 0
        self.ad_span = 80
        self.left_limit = 5600
        self.right_limit = 800
        self.mo_time = 25
        self.range = 20
        self.nbins = 1500
        self.gain = 0.40
        self.speed = 1500.0

        self.on_time = datetime.timedelta(0)
        self.heading = None
        self.recentering = False
        self.centred = None
        self.motoring = False
        self.motor_on = False
        self.clockwise = None
        self.scanning = False
        self.no_params = True
        self.has_cfg = False

        self.conn = Socket(port)

    def __enter__(self):
        """Initializes sonar for first use."""
        self.init()
        return self

    def __exit__(self, type, value, traceback):
        """Cleans up."""
        self.close()

    def init(self):
        """Initializes sonar."""
        self.update()
        self.set()
        self.initialized = True

    def close(self):
        """Closes sonar connection."""
        self.conn.close()
        self.initialized = False

    def get(self, message=None):
        """Sends command and returns reply.

        Args:
            message: Message to expect (default: first to come in).

        Returns:
            Reply.
        """
        while True:
            reply = self.conn.get_reply()
            if reply.id == Message.ALIVE:
                self._update_state(reply)

            if message is not None and reply.id == message:
                return reply

    def send(self, command):
        """Sends command and returns reply.

        Args:
            command: Command to send.
        """
        self.conn.send(command)

    def set(self, adc8on=None, continuous=None, scanright=None, step=None,
            ad_low=None, ad_span=None, left_limit=None, right_limit=None,
            mo_time=None, range=None, nbins=None, gain=None, speed=None):
        self.__set_parameters(
            adc8on=adc8on, continuous=continuous, scanright=scanright,
            step=step, ad_low=ad_low, ad_span=ad_span, left_limit=left_limit,
            right_limit=right_limit, mo_time=mo_time, range=range, nbins=nbins,
            gain=gain, speed=speed
        )

    def __set_parameters(self, **kwargs):
        """Sends Sonar head command with new properties if needed.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            ...
        """
        # Set sonar properties.
        for key, value in kwargs.iteritems():
            if value is not None:
                self.__setattr__(key, value)

        # Construct the HdCtrl bytes to control operation:
        #   Bit 0:  adc8on          0: 4-bit        1: 8-bit
        #   Bit 1:  cont            0: sector-scan  1: continuous
        #   Bit 2:  scanright       0: left         1: right
        #   Bit 3:  invert          0: upright      1: inverted
        #   Bit 4:  motoff          0: on           1: off
        #   Bit 5:  txoff           0: on           1: off (for testing)
        #   Bit 6:  spare           0: default      1: N/A
        #   Bit 7:  chan2           0: default      1: N/A
        #   Bit 8:  raw             0: N/A          1: default
        #   Bit 9:  hasmot          0: lol          1: has a motor (always)
        #   Bit 10: applyoffset     0: default      1: heading offset
        #   Bit 11: pingpong        0: default      1: side-scanning sonar
        #   Bit 12: stareLLim       0: default      1: N/A
        #   Bit 13: ReplyASL        0: N/A          1: default
        #   Bit 14: ReplyThr        0: default      1: N/A
        #   Bit 15: IgnoreSensor    0: default      1: emergencies
        hd_ctrl = bitstring.pack(
            "bool, bool, bool, bool, 0b000011001000",
            self.adc8on, self.continuous, self.scanright, self.inverted
        )

        # Set the sonar type: 0x11 for DST.
        hd_type = bitstring.pack("0x11")

        # TX/RX transmitter constants: N/A to DST so fill with 15 zero bytes.
        # TX pulse length: N/A to DST so fill with 2 zero bytes.
        tx_rx = bitstring.BitStream(144)

        # Range scale does not control the sonar, only provides a way to note
        # the current settings in a human readable format.
        # The lower 14 bits are the range scale * 10 units and the higher 2
        # bits are coded units:
        #   0: meters
        #   1: feet
        #   2: fathoms
        #   3: yards
        range_scale = bitstring.pack("uintle:16", int(self.range * 10))

        # Left/right angles limits are in 1/16th of a gradian.
        left_limit = bitstring.pack("uintle:16", self.left_limit)
        right_limit = bitstring.pack("uintle:16", self.right_limit)

        # Set the mapping of the received sonar echo amplitudes.
        # If the ADC is set to 8-bit, MAX = 255 else MAX = 15.
        # ADLow = MAX * low / 80 where low is the desired minimum amplitude.
        # ADSpan = MAX * range / 80 where range is the desired amplitude range.
        # The full range is between ADLow and ADLow + ADSpan.
        MAX_SIZE = 255 if self.adc8on else 15
        ad_low = bitstring.pack("uint:8", int(MAX_SIZE * self.ad_low / 80))
        ad_span = bitstring.pack("uint:8", int(MAX_SIZE * self.ad_span / 80))

        # Set the initial gain of each channel of the sonar receiver.
        # The gain ranges from 0 to 210.
        _mapped_gain = int(self.gain * 210)
        gain = bitstring.pack("uint:8, uint:8", _mapped_gain, _mapped_gain)

        # Slope setting is not applicable to DST: fill 4 bytes with zeroes.
        slope = bitstring.BitStream(32)

        # Set the high speed limit of the motor in units of 10 microseconds.
        mo_time = bitstring.pack("uint:8", self.mo_time)

        # Set the step angle size in 1/16th of a gradian.
        #   32: low resolution
        #   16: medium resolution
        #   8: high resolution
        step = bitstring.pack("uint:8", self.step)

        # ADInterval defines the sampling interval of each bin and is in units
        # of 640 nanoseconds.
        nbins = bitstring.pack("uintle:16", self.nbins)
        _interval = int(2 * self.range / self.speed / self.nbins / 640e-9)
        ad_interval = bitstring.pack("uintle:16", _interval)

        # Factory defaults. Don't ask.
        max_ad_buf = bitstring.pack("uintle:16", 500)
        lockout = bitstring.pack("uintle:16", 100)
        minor_axis = bitstring.pack("uintle:16", 1600)
        major_axis = bitstring.pack("uint:8", 1)

        # Ctl2 is for testing.
        ctl2 = bitstring.pack("uint:8", 0)

        # Special devices setting. Should be left blank.
        scanz = bitstring.pack("uint:8, uint:8", 0, 0)

        # Order bitstream.
        bitstream = (
            hd_ctrl, hd_type, tx_rx, range_scale, left_limit, right_limit,
            ad_span, ad_low, gain, slope, mo_time, step, ad_interval, nbins,
            max_ad_buf, lockout, minor_axis, major_axis, ctl2, scanz
        )

        payload = bitstring.BitStream()
        for chunk in bitstream:
            payload.append(chunk)

        self.conn.send(Message.HEAD_COMMAND, payload)

    def scan(self, **kwargs):
        """Sends scan command.

        This method is blocking but calls feedback_callback at every reply with
        the heading and a new dataset and complete_callback with the current
        heading when scan is complete.

        Args:
            feedback_callback: Callback for feedback.
                Called with args=(heading, bins)
                where heading is an int in 1/16th Gradians
                and bins is an int array with the intensity at every bin.
            complete_callback: Callback on completion.
                Called with args=(heading,)
                where heading is an int in 1/16th Gradians.
            kwargs: Key-word arguments to pass to update before scanning.
        """
        # Update scan settings.
        if kwargs:
            self.set(**kwargs)

        _time = datetime.datetime.now().time()
        current_millis = int(datetime.timedelta(
            _time.hour, _time.minute, _time.second, _time.microsecond
        ).total_seconds() * 1000)
        print "poop"
        payload = bitstring.pack("uintle:32", current_millis)
        print "lol"
        self.conn.send(Message.SEND_DATA, payload)
        print "ca va"

    def reboot(self):
        """Reboots Sonar."""
        self.conn.send(Message.REBOOT)
        self.update()

    def update(self):
        """Updates Sonar states from mtAlive message."""
        reply = self.conn.get_reply(expected=Message.ALIVE)
        self._update_state(reply)

    def _update_state(self, alive):
        """Updates Sonar states from mtAlive message.

        Args:
            alive: mtAlive reply.
        """
        payload = alive.payload
        payload.bytepos = 1

        ms = struct.unpack("<L", payload.read(32).tobytes())[0]
        self.on_time = datetime.timedelta(microseconds=ms * 1000)
        self.heading = struct.unpack("<h", payload.read(16).tobytes())[0]

        head_inf = payload.read(8)
        self.recentering = head_inf[0]
        self.centred = head_inf[1]
        self.motoring = head_inf[2]
        self.motor_on = head_inf[3]
        self.clockwise = head_inf[4]
        self.scanning = head_inf[5]
        self.no_params = head_inf[6]
        self.has_cfg = head_inf[7]


if __name__ == '__main__':
    with Sonar("/dev/tty.usbserial") as sonar:
        print "ON TIME:", sonar.on_time
        print "REBOOTING SONAR..."
        sonar.reboot()
        print "ON TIME:", sonar.on_time
        sonar.scan()
