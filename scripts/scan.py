#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

import rospy
from tritech_micron import TritechMicron

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


if __name__ == "__main__":
    rospy.init_node("tritech_micron", log_level=rospy.DEBUG)

    def feedback_callback(sonar, heading, bins):
        print TritechMicron.to_sonar_angles(heading), bins
        raw_input("Continue")

    def complete_callback(sonar, heading):
        print "DONE", heading

    with TritechMicron(port="/dev/ttyUSB0") as sonar:
        sonar.scan(feedback_callback, complete_callback)
