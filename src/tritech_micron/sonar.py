# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

from socket import Socket

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
        return self.conn.get_reply(expected=command)

    def update(self, adc8on=None, continuous=None, scanright=None, step=None,
               ad_low=None, ad_span=None, left_limit=None, right_limit=None,
               mo_time=None, range_scale=None, ad_interval=None, nbins=None):
        """Send Sonar head command with new properties if needed.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            ...
        """
        pass

    def scan(self, feedback_callback, complete_callback, **kwargs):
        """Send scan command.

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
        self.update(**kwargs)

        pass
