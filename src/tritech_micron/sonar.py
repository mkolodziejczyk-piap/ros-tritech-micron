# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

import math
import rospy
import datetime
import bitstring
import exceptions
from socket import Socket
from timeout import timeout
from messages import Message

__author__ = "Anass Al-Wohoush"
__version__ = "0.7.0"


class Resolution(object):

    """Sonar mechanical resolution enumeration.

    The mechanical resolution is the angle the step motor rotates per scan line
    in units of 1/16th of a gradian. A higher resolution slows down the scan.
    Set as such:

        with TritechMicron() as sonar:
            sonar.set(step=Resolution.LOW)
    """

    LOW = 32
    MEDIUM = 16
    HIGH = 8
    ULTIMATE = 4


class TritechMicron(object):

    """Tritech Micron Sonar.

    In order for attribute changes to immediately be reflected on the device,
    use the set() method with the appropriate keyword.

    For example, to setup a 20 m sector scan:

        with TritechMicron() as sonar:
            sonar.set(continuous=False, range=20)

    Attributes:
        ad_high: Amplitude in db to be mapped to max intensity (0-80 db).
        ad_low: Amplitude in dB to be mapped to 0 intensity (0-80 db).
        adc8on: True if 8-bit resolution ADC, otherwise 4-bit.
        centred: Whether the sonar motor is centred.
        clock: Sonar time of the day.
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

        # Additional information.
        self.clock = None
        self._time_offset = datetime.timedelta(0)

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
            except OSError as e:
                raise exceptions.SonarNotFound(self.port, e)

        # Update properties.
        rospy.loginfo("Initializing sonar on %s", self.port)
        self.initialized = True
        self.send(Message.REBOOT)
        self.update()

        # Verify version.
        self.send(Message.SEND_VERSION)
        self.get(Message.VERSION_DATA)

        # Verify BB User Data.
        self.send(Message.SEND_BB_USER)
        self.get(Message.BB_USER_DATA)
        self.get(Message.FPGA_CAL_DATA)
        self.get(Message.FPGA_VERSION_DATA)

        # Set default properties.
        self.set(
            adc8on=True, continuous=True, scanright=True, step=Resolution.LOW,
            ad_low=0, ad_high=80, left_limit=5600, right_limit=800, mo_time=25,
            range=20, nbins=200, gain=0.40, speed=1500.0
        )

        rospy.loginfo("Sonar is ready for use")

    def close(self):
        """Closes sonar connection."""
        self.send(Message.REBOOT)
        self.conn.close()
        self.initialized = False
        rospy.loginfo("Closed sonar socket")

    def get(self, message=None, wait=2):
        """Sends command and returns reply.

        Args:
            message: Message to expect (default: first to come in).
            wait: Seconds to wait until received (default: 2).

        Returns:
            Reply if successful, None otherwise.

        Raises:
            SonarNotInitialized: Attempt reading serial without opening port.
        """
        if message:
            name = Message.to_string(message)
            rospy.loginfo("Waiting for %s message", name)
        if not self.initialized:
            raise exceptions.SonarNotInitialized(message)

        try:
            with timeout(seconds=wait):
                while True:
                    reply = self.conn.get_reply()
                    if reply.id == Message.ALIVE:
                        self.__update_state(reply)
                    if message is None:
                        return reply
                    if reply.id == message:
                        rospy.loginfo("Found %s message", name)
                        return reply
                    elif reply.id != Message.ALIVE:
                        rospy.logwarn(
                            "Received unexpected %s message",
                            reply.type
                        )
        except (exceptions.PacketCorrupted, exceptions.TimeoutError) as e:
            if message:
                rospy.logerr("Failed to get %s message: %r", name, e)
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
        rospy.logwarn("Setting parameters...")

        # Set and compare sonar properties.
        necessary = not self.has_cfg or self.no_params
        only_reverse = not necessary
        for key, value in kwargs.iteritems():
            if value is not None:
                if self.__getattribute__(key) != value:
                    self.__setattr__(key, value)
                    necessary = True
                    if key != "scanright":
                        only_reverse = False

        # Return if unnecessary.
        if not necessary:
            rospy.loginfo("Parameters are already set")
            return

        # Return if only switching the motor's direction is necessary.
        if only_reverse:
            rospy.loginfo("Only reversing direction")
            self.scanright = not self.scanright
            return self.reverse()

        self._log_properties()

        # This device is not Dual Channel so skip the “V3B” Gain Parameter
        # block: 0x01 for normal, 0x1D for extended V3B Gain Parameters.
        v3b = bitstring.pack("0x01")

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
            "bool, bool, bool, bool, 0b000011000100",
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
        _left_angle = TritechMicron.to_sonar_angles(self.left_limit)
        _right_angle = TritechMicron.to_sonar_angles(self.right_limit)
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

        # Slope setting is N/A to DST: fill 4 bytes with zeroes.
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
        _interval = 2 * self.range / self.speed / self.nbins / 640e-9
        _interval = _interval + 1 if _interval % 2 else _interval
        ad_interval = bitstring.pack("uintle:16", int(_interval))

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
            v3b, hd_ctrl, hd_type, tx_rx, range_scale, left_limit, right_limit,
            ad_span, ad_low, gain, slope, mo_time, step, ad_interval, nbins,
            max_ad_buf, lockout, minor_axis, major_axis, ctl2, scanz
        )

        payload = bitstring.BitStream()
        for chunk in bitstream:
            payload.append(chunk)

        self.send(Message.HEAD_COMMAND, payload)

        # Wait until sonar acknowledges properties.
        while not self.has_cfg or self.no_params:
            rospy.logdebug(
                "Waiting for configuration: (HAS CFG: %s, NO PARAMS: %s)",
                self.has_cfg, self.no_params
            )
            self.update()

        rospy.logwarn("Parameters are set")

    def reverse(self):
        """Instantaneously reverse scan direction."""
        payload = bitstring.pack("0x0F")
        self.send(Message.HEAD_COMMAND, payload)
        self.scanright = not self.scanright

    def scan(self, feedback_callback, complete_callback, **kwargs):
        """Sends scan command.

        This method is blocking but calls feedback_callback at every reply with
        the heading and a new dataset and complete_callback with the current
        heading when scan is complete.

        To stop a scan midway, simply call the preempt().

        Args:
            feedback_callback: Callback for feedback.
                Called with args=(sonar, heading, bins)
                where heading is an int in radians
                and bins is an int array with the intensity at every bin.
            complete_callback: Callback on completion or halt.
                Called with args=(sonar, heading,)
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

        def ping():
            """Commands the sonar to ping."""
            # Get current time in milliseconds.
            now = datetime.datetime.now()
            current_time = datetime.timedelta(
                hours=now.hour, minutes=now.minute,
                seconds=now.second, microseconds=0
            )
            payload = bitstring.pack(
                "uintle:32",
                current_time.total_seconds() * 1000
            )

            # Reset offset for on time.
            self._time_offset = current_time - self.on_time

            # Send command.
            self.send(Message.SEND_DATA, payload)

        self.preempted = False
        while not self.preempted:
            # Pings the sonar.
            ping()

            # Queues the next ping for quicker scanning.
            ping()

            # Get the current heading data.
            head_data = self.get(Message.HEAD_DATA, wait=1)
            if not head_data:
                continue

            data = head_data.payload

            # Get the total number of bytes.
            count = data.read(16).uintle
            rospy.logdebug("Byte count is %d", count)

            # The device type should be 0x11 for a DST Sonar.
            device_type = data.read(8)
            if device_type.uint != 0x11:
                rospy.logerr("Unexpected device type: %s", device_type.hex)
            else:
                rospy.logdebug("Received device type: %s", device_type.hex)

            # Get the head status byte:
            #   Bit 0:  'HdPwrLoss'. Head is in Reset Condition.
            #   Bit 1:  'MotErr'. Motor has lost sync, re-send Parameters.
            #   Bit 2:  'PrfSyncErr'. Always 0.
            #   Bit 3:  'PrfPingErr'. Always 0.
            #   Bit 4:  Whether adc8on is enabled.
            #   Bit 5:  RESERVED (ignore).
            #   Bit 6:  RESERVED (ignore).
            #   Bit 7:  Message appended after last packet data reply.
            _head_status = data.read(8).bin
            rospy.logdebug("Head status byte is %s", _head_status)

            # Get the sweep code. Its value should correspond to:
            #   0: Scanning normal.
            #   1: Scan at left limit.
            #   2: Scan at right limit.
            #   3: RESERVED (ignore).
            #   4: RESERVED (ignore)
            #   5: Scan at center position.
            sweep = data.read(8).uint
            rospy.logdebug("Sweep code is %d", sweep)

            # Get the HdCtrl bytes to control operation:
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
            # Should be the same as what was sent.
            hd_ctrl = data.read(16)
            self.adc8on = hd_ctrl[0]
            self.continuous = hd_ctrl[1]
            self.scanright = hd_ctrl[2]
            self.inverted = hd_ctrl[3]
            self.motor_on = not hd_ctrl[4]
            rospy.logdebug("Head control bytes are %s", hd_ctrl.bin)

            # Range scale.
            # The lower 14 bits are the range scale * 10 units and the higher 2
            # bits are coded units:
            #   0: meters
            #   1: feet
            #   2: fathoms
            #   3: yards
            # Only the metric system is implemented for now, because it is
            # better.
            self.range = data.read(16).uintle / 10.0
            rospy.logdebug("Range scale is %f", self.range)

            # TX/RX transmitter constants: N/A to DST.
            tx_rx = data.read(32)

            # The gain ranges from 0 to 210.
            self.gain = data.read(8).uintle / 210.0
            rospy.logdebug("Gain is %f", self.gain)

            # Slope setting is N/A to DST.
            slope = data.read(16)

            # If the ADC is set to 8-bit, MAX = 255 else MAX = 15.
            # ADLow = MAX * low / 80 where low is the desired minimum
            #   amplitude.
            # ADSpan = MAX * range / 80 where range is the desired amplitude
            #   range.
            # The full range is between ADLow and ADLow + ADSpan.
            MAX_SIZE = 255 if self.adc8on else 15
            ad_span = data.read(8).uintle
            self.ad_low = data.read(8).uintle
            min_intensity = self.ad_low * 80.0 / MAX_SIZE
            span_intensity = ad_span * 80.0 / MAX_SIZE
            self.ad_high = min_intensity + span_intensity
            rospy.logdebug("AD range is %f to %f", self.ad_low, self.ad_high)

            # Heading offset is ignored.
            heading_offset = TritechMicron.to_radians(data.read(16).uint)
            rospy.logdebug("Heading offset is %f", heading_offset)

            # ADInterval defines the sampling interval of each bin and is in
            # units of 640 nanoseconds.
            ad_interval = data.read(16).uintle
            rospy.logdebug("AD interval is %d", ad_interval)

            # Left/right angles limits are in 1/16th of a gradian.
            self.left_limit = TritechMicron.to_radians(data.read(16).uintle)
            self.right_limit = TritechMicron.to_radians(data.read(16).uintle)
            rospy.logdebug(
                "Limits are %f to %f",
                self.left_limit, self.right_limit
            )

            # Step angle size.
            self.step = TritechMicron.to_radians(data.read(8).uint)
            rospy.logdebug("Step size is %f", self.step)

            # Heading is in units of 1/16th of a gradian.
            self.heading = TritechMicron.to_radians(data.read(16).uintle)
            rospy.loginfo("Heading is now %f", self.heading)

            # Dbytes is the number of bytes with data to follow.
            dbytes = data.read(16).uintle
            rospy.logdebug("DBytes is %d", dbytes)

            # Get bins.
            if self.adc8on:
                bins = [data.read(8).uint for i in range(dbytes)]
            else:
                bins = [data.read(4).uint for i in range(dbytes * 2)]

            # Run feedback callback.
            feedback_callback(self, self.heading, bins)

            # Proceed or not.
            if not self.scanright and sweep == 1:
                rospy.logwarn("Reached left limit")
            elif self.scanright and sweep == 2:
                rospy.logwarn("Reached right limit")

        # Run completion callback.
        complete_callback(self, self.heading)

    def preempt(self):
        """Preempts a scan in progress."""
        rospy.logwarn("Preempting scan...")
        self.preempted = True

    def reboot(self):
        """Reboots Sonar.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        rospy.logwarn("Rebooting sonar...")
        self.send(Message.REBOOT)
        self.open()

    def update(self):
        """Updates Sonar states from mtAlive message.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        self.get(Message.ALIVE)

    def __update_state(self, alive):
        """Updates Sonar states from mtAlive message.

        Args:
            alive: mtAlive reply.
        """
        payload = alive.payload
        payload.bytepos = 1

        micros = payload.read(32).uintle * 1000
        self.clock = datetime.timedelta(microseconds=micros)
        self.on_time = self.clock - self._time_offset
        self.heading = TritechMicron.to_radians(payload.read(16).uintle)

        head_inf = payload.read(8)
        self.recentering = head_inf[0]
        self.centred = head_inf[1]
        self.motoring = head_inf[2]
        self.motor_on = head_inf[3]
        self.scanright = head_inf[4]
        self.scanning = head_inf[5]
        self.no_params = head_inf[6]
        self.has_cfg = head_inf[7]

        rospy.logdebug("ON TIME:     %s", self.on_time)
        rospy.logdebug("RECENTERING: %s", self.recentering)
        rospy.logdebug("CENTRED:     %s", self.centred)
        rospy.logdebug("MOTORING:    %s", self.motoring)
        rospy.logdebug("MOTOR ON:    %s", self.motor_on)
        rospy.logdebug("CLOCKWISE:   %s", self.scanright)
        rospy.logdebug("SCANNING:    %s", self.scanning)
        rospy.logdebug("NO PARAMS:   %s", self.no_params)
        rospy.logdebug("HAS CFG:     %s", self.has_cfg)

    def _log_properties(self):
        rospy.logwarn("CONTINUOUS:  %s", self.continuous)
        rospy.logwarn("LEFT LIMIT:  %s", self.left_limit)
        rospy.logwarn("RIGHT LIMIT: %s", self.right_limit)
        rospy.logwarn("STEP SIZE:   %s", self.step)
        rospy.logwarn("N BINS:      %s", self.nbins)
        rospy.logwarn("RANGE:       %s m", self.range)
        rospy.logwarn("INVERTED:    %s", self.inverted)
        rospy.logwarn("AD HIGH:     %s dB", self.ad_high)
        rospy.logwarn("AD LOW:      %s dB", self.ad_low)
        rospy.logwarn("ADC 8 ON:    %s", self.adc8on)
        rospy.logwarn("GAIN:        %s%%", self.gain * 100)
        rospy.logwarn("MOTOR TIME:  %s us", self.mo_time)
        rospy.logwarn("CLOCKWISE:   %s", self.scanright)
        rospy.logwarn("SPEED:       %s m/s", self.speed)

    @classmethod
    def to_sonar_angles(cls, rad):
        """Converts radians to units of 1/16th of a gradian.

        Args:
            rad: Angle in radians.

        Returns:
            Integral angle in units of 1/16th of a gradian.
        """
        return int(rad * 3200 / math.pi) % 6400

    @classmethod
    def to_radians(cls, angle):
        """Converts units of 1/16th of a gradian to radians.

        Args:
            angle: Angle in units of 1/16th of a gradian.

        Returns:
            Angle in radians.
        """
        return angle / 3200.0 * math.pi
