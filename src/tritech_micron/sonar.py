# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

import struct
import datetime
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

    def __init__(self, port="/dev/sonar"):
        """Constructs Sonar object.

        Args:
            port: Serial port.
        """
        self.initialized = False

        self.adc8on = True
        self.continuous = True
        self.scanright = True

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
        self.update()
        return self

    def __exit__(self, type, value, traceback):
        """Cleans up."""
        self.close()

    def init(self):
        """Initializes sonar."""
        self.update()
        self.initialized = True

    def close(self):
        """Closes sonar connection."""
        self.conn.close()
        self.initialized = False

    def get(self, command):
        """Sends command and returns reply.

        Args:
            command: Command to send.

        Returns:
            Reply.
        """
        self.conn.send(command)
        reply = self.conn.get_reply()
        if reply.id == Message.ALIVE:
            self._update_state(reply)
        return reply

    def params(self, adc8on=None, continuous=None, scanright=None, step=None,
               ad_low=None, ad_span=None, left_limit=None, right_limit=None,
               mo_time=None, range_scale=None, ad_interval=None, nbins=None):
        """Sends Sonar head command with new properties if needed.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            ...
        """
        pass

    def scan(self, feedback_callback, complete_callback, **kwargs):
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
        self.params(**kwargs)

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
        print sonar.get(Message.ALIVE)
