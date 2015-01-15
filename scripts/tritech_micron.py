# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"

import rospy
from tritech_micron.sonar import Sonar


if __name__ == '__main__':
    with Sonar as sonar:
        rospy.spin()
