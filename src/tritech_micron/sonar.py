# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

import math
import datetime
import bitstring
import exceptions
from socket import Socket
from messages import Message

__author__ = "Anass Al-Wohoush"
__version__ = "0.7.0"


class Resolution(object):

    """Sonar mechanical resolution enumeration.

    The mechanical resolution is the angle the step motor rotates per scan line
    in units of 1/16th of a gradian. A higher resolution slows down the scan.
    Set as such:

        with Sonar() as sonar:
            sonar.set(step=Resolution.LOW)
    """

    LOW = 32
    MEDIUM = 16
    HIGH = 8
    ULTIMATE = 4


class Sonar(object):

    """Tritech Micron Sonar.

    In order for attribute changes to immediately be reflected on the device,
    use the set() method with the appropriate keyword.

    For example, to setup a 20 m sector scan:

        with Sonar() as sonar:
            sonar.set(continuous=False, range=20)

    Attributes:
        ad_high: Amplitude in db to be mapped to max intensity (0-80 db).
        ad_low: Amplitude in dB to be mapped to 0 intensity (0-80 db).
        adc8on: True if 8-bit resolution ADC, otherwise 4-bit.
        centred: Whether the sonar motor is centred.
        conn: Serial connection.
        continuous: True if continuous scan, otherwise sector scan.
        gain: Initial gain percentage (0.00-1.00).
        has_cfg: Whether the sonar has acknowledged with mtBBUserData.
        heading: Current sonar heading in radians.
        initialized: Whether the sonar has been initialized with parameters.
        inverted: Whether the sonar is mounted upside down.
        left_limit: Left limit of sector scan in radians.
        mo_time: High speed limit of the motor in units of 10 microseconds.
        motoring: Whether the sonar motor is moving.
        motor_on: Whether device is powered and motor is primed.
        nbins: Number of bins per scan line.
        no_params: Whether the sonar needs parameters before it can scan.
        on_time: Sonar up time.
        port: Serial port.
        range: Scan range in meters.
        recentering: Whether the sonar is recentering its motor.
        right_limit: Right limit of sector scans in radians.
        scanning: Whether the sonar is scanning.
        scanright: Whether the sonar scanning direction is clockwise.
        speed: Speed of sound in medium.
        step: Mechanical resolution (Resolution enumeration).
    """

    def __init__(self, port="/dev/sonar", inverted=False):
        """Constructs Sonar object.

        Args:
            port: Serial port.
            inverted: Whether the sonar is mounted upside down.
        """

        self.port = port
        self.conn = None
        self.initialized = False

        # Parameters.
        self.ad_high = None
        self.ad_low = None
        self.adc8on = None
        self.continuous = None
        self.gain = None
        self.inverted = inverted
        self.left_limit = None
        self.mo_time = None
        self.nbins = None
        self.range = None
        self.right_limit = None
        self.scanright = None
        self.speed = None
        self.step = None

        # Head info.
        self.centred = None
        self.has_cfg = None
        self.heading = None
        self.motor_on = None
        self.motoring = None
        self.no_params = None
        self.on_time = datetime.timedelta(0)
        self.recentering = None
        self.scanning = None

    def __enter__(self):
        """Initializes sonar for first use.

        Raises:
            SonarNotFound: Sonar port could not be opened.
        """
        self.open()
        return self

    def __exit__(self, type, value, traceback):
        """Cleans up."""
        self.close()

    def open(self):
        """Initializes sonar connection and sets default properties.

        Raises:
            SonarNotFound: Sonar port could not be opened.
        """
        if not self.conn:
            try:
                self.conn = Socket(self.port)
            except OSError:
                raise exceptions.SonarNotFound(self.port)

        # Update properties.
        self.initialized = True
        self.update()

        # Wait until sonar is done centering.
        while not self.centred or self.motoring:
            self.update()

        # Set default properties.
        self.set(
            adc8on=True, continuous=True, scanright=True, step=Resolution.LOW,
            ad_low=0, ad_high=80, left_limit=5600, right_limit=800, mo_time=25,
            range=20, nbins=200, gain=0.40, speed=1500.0, inverted=True
        )

        # Wait until sonar acknowledges properties.
        while not self.has_cfg or self.no_params:
            self.update()

    def close(self):
        """Closes sonar connection."""
        self.send(Message.REBOOT)
        self.conn.close()
        self.initialized = False

    def get(self, message=None):
        """Sends command and returns reply.

        Args:
            message: Message to expect (default: first to come in).

        Returns:
            Reply if successful, None otherwise.

        Raises:
            SonarNotInitialized: Attempt reading serial without opening port.
        """
        if not self.initialized:
            raise exceptions.SonarNotInitialized(message)

        try:
            while True:
                reply = self.conn.get_reply()
                if reply.id == Message.ALIVE:
                    self.__update_state(reply)
                if message is None or reply.id == message:
                    return reply
        except exceptions.PacketCorrupted:
            return None

    def send(self, command, payload=None):
        """Sends command and returns reply.

        Args:
            command: Command to send.

        Raises:
            SonarNotInitialized: Attempt sending command without opening port.
        """
        if not self.initialized:
            raise exceptions.SonarNotInitialized(command, payload)

        self.conn.send(command, payload)

    def set(self, adc8on=None, continuous=None, scanright=None, step=None,
            ad_low=None, ad_high=None, left_limit=None, right_limit=None,
            mo_time=None, range=None, nbins=None, gain=None, speed=None,
            inverted=None):
        """Sends Sonar head command with new properties if needed.

        Only the parameters specified will be modified.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            ad_low: Amplitude in dB to be mapped to 0 intensity (0-80 db).
            ad_high: Amplitude in db to be mapped to max intensity (0-80 db).
            adc8on: True if 8-bit resolution ADC, otherwise 4-bit.
            continuous: True if continuous scan, otherwise sector scan.
            gain: Initial gain percentage (0.00-1.00).
            inverted: Whether the sonar is mounted upside down.
            left_limit: Left limit of sector scan in radians.
            mo_time: High speed limit of the motor in units of 10 microseconds.
            nbins: Number of bins per scan line.
            range: Scan range in meters.
            right_limit: Right limit of sector scans in radians.
            scanright: Whether the sonar scanning direction is clockwise.
            speed: Speed of sound in medium.
            step: Mechanical resolution (Resolution enumeration).

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        if not self.initialized:
            raise exceptions.SonarNotInitialized()

        self.__set_parameters(
            adc8on=adc8on, continuous=continuous, scanright=scanright,
            step=step, ad_low=ad_low, ad_high=ad_high, left_limit=left_limit,
            right_limit=right_limit, mo_time=mo_time, range=range, nbins=nbins,
            gain=gain, speed=speed, inverted=inverted
        )

    def __set_parameters(self, **kwargs):
        """Sends Sonar head command to set sonar properties.

        Only the parameters specified will be modified.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            See set().
        """
        # Set and compare sonar properties.
        necessary, only_reverse = False, True
        for key, value in kwargs.iteritems():
            if value is not None:
                if self.__getattribute__(key) != value:
                    self.__setattr__(key, value)
                    necessary = True
                    if key != "scanright":
                        only_reverse = False

        # Return if unnecessary.
        if not necessary and self.initialized:
            return

        # Return if only switching the motor's direction is necessary.
        if only_reverse:
            self.scanright = not self.scanright
            return self.reverse()

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
        # Only the metric system is implemented for now, because it is better.
        range_scale = bitstring.pack("uintle:16", int(self.range * 10))

        # Left/right angles limits are in 1/16th of a gradian.
        _left_angle = Sonar._to_sonar_angles(self.left_limit)
        _right_angle = Sonar._to_sonar_angles(self.right_limit)
        left_limit = bitstring.pack("uintle:16", _left_angle)
        right_limit = bitstring.pack("uintle:16", _right_angle)

        # Set the mapping of the received sonar echo amplitudes.
        # If the ADC is set to 8-bit, MAX = 255 else MAX = 15.
        # ADLow = MAX * low / 80 where low is the desired minimum amplitude.
        # ADSpan = MAX * range / 80 where range is the desired amplitude range.
        # The full range is between ADLow and ADLow + ADSpan.
        MAX_SIZE = 255 if self.adc8on else 15
        ad_low = bitstring.pack("uint:8", int(MAX_SIZE * self.ad_low / 80))
        _ad_diff = self.ad_high - self.ad_low
        ad_span = bitstring.pack("uint:8", int(MAX_SIZE * _ad_diff / 80))

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

        self.send(Message.HEAD_COMMAND, payload)

    def reverse(self):
        """Instantaneously reverse scan direction."""
        payload = bitstring.pack("0x0F")
        self.send(Message.HEAD_COMMAND, payload)
        self.scanright = not self.scanright

    def scan(self, **kwargs):
        """Sends scan command.

        This method is blocking but calls feedback_callback at every reply with
        the heading and a new dataset and complete_callback with the current
        heading when scan is complete.

        To stop a scan midway, simply have the feedback_callback return False.

        Args:
            feedback_callback: Callback for feedback.
                Called with args=(heading, bins)
                where heading is an int in radians
                and bins is an int array with the intensity at every bin.
                This callback should return True if the scan should proceed and
                False to halt it.
            complete_callback: Callback on completion or halt.
                Called with args=(heading,)
                where heading is an int in radians.
            kwargs: Key-word arguments to pass to update before scanning.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
            SonarNotConfigured: Sonar is not configured for scanning.
        """
        # Verify sonar is ready to scan.
        self.update()
        if self.no_params or not self.has_cfg:
            raise exceptions.SonarNotConfigured(self.no_params, self.has_cfg)

        # Update scan settings.
        if kwargs:
            self.set(**kwargs)

        # Compute current time in milliseconds.
        t = datetime.datetime.now().time()
        current_millis = int(
            ((t.hour * 60 + t.minute) * 60 + t.second) * 1000 +
            t.microsecond / 1000
        )
        payload = bitstring.pack("uintle:32", current_millis)

        # Send command.
        self.send(Message.SEND_DATA, payload)

    def reboot(self):
        """Reboots Sonar.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        self.send(Message.REBOOT)
        self.open()

    def update(self):
        """Updates Sonar states from mtAlive message.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        reply = self.get(Message.ALIVE)
        self.__update_state(reply)

    def __update_state(self, alive):
        """Updates Sonar states from mtAlive message.

        Args:
            alive: mtAlive reply.
        """
        payload = alive.payload
        payload.bytepos = 1

        micros = payload.read(32).uintle * 1000
        self.on_time = datetime.timedelta(microseconds=micros)
        self.heading = Sonar._to_radians(payload.read(16).uintle)

        head_inf = payload.read(8)
        self.recentering = head_inf[0]
        self.centred = head_inf[1]
        self.motoring = head_inf[2]
        self.motor_on = head_inf[3]
        self.clockwise = head_inf[4]
        self.scanning = head_inf[5]
        self.no_params = head_inf[6]
        self.has_cfg = head_inf[7]

    @classmethod
    def _to_sonar_angles(cls, rad):
        """Converts radians to units of 1/16th of a gradian.

        Args:
            rad: Angle in radians.

        Returns:
            Integral angle in units of 1/16th of a gradian.
        """
        return int(rad * 3200 / math.pi) % 6400

    @classmethod
    def _to_radians(cls, angle):
        """Converts units of 1/16th of a gradian to radians.

        Args:
            rad: Angle in units of 1/16th of a gradian.

        Returns:
            Angle in radians.
        """
        return angle / 3200.0 * math.pi


if __name__ == '__main__':
    with Sonar("/dev/tty.usbserial") as sonar:
        print "ON TIME:", sonar.on_time
        print "REBOOTING SONAR..."
        sonar.reboot()
        print "ON TIME:", sonar.on_time
        sonar.scan()
