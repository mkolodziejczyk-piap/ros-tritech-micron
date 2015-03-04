# -*- coding: utf-8 -*-

"""Tritech Micron Sonar."""

import rospy
from tritech_micron.sonar import Sonar

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


if __name__ == '__main__':
    with Sonar() as sonar:
        rospy.spin()
