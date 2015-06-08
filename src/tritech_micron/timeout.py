# -*- coding: utf-8 -*-

"""Tritech Micron Sonar timeout."""

import os
import errno
import signal
from exceptions import TimeoutError
from contextlib import contextmanager

__author__ = "Anass Al-Wohoush"


@contextmanager
def timeout(seconds):
    """Times out after n seconds if process hasn't completed.

   For example, to timeout after 10 seconds:

        try:
            with timeout(seconds=10):
                # Some blocking task...
        except TimeoutError:
            # Oops...

    Args:
        seconds: Seconds until timeout.

    Raises:
        TimeoutError: If function times out.
    """
    def handle_timeout(signum, frame):
        raise TimeoutError(os.strerror(errno.ETIME))

    # Set up signal interrupt.
    signal.signal(signal.SIGALRM, handle_timeout)
    signal.alarm(seconds)

    # Yield context to user.
    yield

    # Stop interrupt if complete.
    signal.alarm(0)
