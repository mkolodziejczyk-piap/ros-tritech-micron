#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron CSV to PointCloud.

This script converts a CSV logged by the Tritech Windows utility to ROS
messages.

Note: This does not currently support decoding Tritech's V4LOG file format, so
you must first convert them to CSV using Tritech's Windows utility.
"""

import csv
import sys
import rospy
import bitstring
from datetime import datetime
from sensor_msgs.msg import PointCloud
from tritech_micron import TritechMicron
from geometry_msgs.msg import PoseStamped
from tritech_micron.msg import TritechMicronConfig
from scan import to_config, to_pointcloud, to_posestamped

__author__ = "Anass Al-Wohoush, Max Krogius"


class Slice(object):

    """Scan slice.

    Attributes:
        ad_high: Amplitude in db to be mapped to max intensity (0-80 db).
        ad_low: Amplitude in dB to be mapped to 0 intensity (0-80 db).
        adc8on: True if 8-bit resolution ADC, otherwise 4-bit.
        continuous: True if continuous scan, otherwise sector scan.
        gain: Initial gain percentage (0.00-1.00).
        heading: Current sonar heading in radians.
        inverted: Whether the sonar is mounted upside down.
        left_limit: Left limit of sector scan in radians.
        nbins: Number of bins per scan line.
        range: Scan range in meters.
        right_limit: Right limit of sector scans in radians.
        scanright: Whether the sonar scanning direction is clockwise.
        step: Mechanical resolution (Resolution enumeration).
    """

    def __init__(self, row):
        """Constructs Slice object.

        Args:
            row: Current row as column array from CSV log.
        """
        # Extract timestamp.
        self.timestamp = datetime.strptime(row[1], "%H:%M:%S.%f")

        # Scan angles information.
        self.left_limit = TritechMicron.to_radians(int(row[10]))
        self.right_limit = TritechMicron.to_radians(int(row[11]))
        self.step = TritechMicron.to_radians(int(row[12]))
        self.heading = TritechMicron.to_radians(int(row[13]))
        rospy.loginfo("Heading is now %f", self.heading)

        # Get the head status byte:
        #   Bit 0:  'HdPwrLoss'. Head is in Reset Condition.
        #   Bit 1:  'MotErr'. Motor has lost sync, re-send Parameters.
        #   Bit 2:  'PrfSyncErr'. Always 0.
        #   Bit 3:  'PrfPingErr'. Always 0.
        #   Bit 4:  Whether adc8on is enabled.
        #   Bit 5:  RESERVED (ignore).
        #   Bit 6:  RESERVED (ignore).
        #   Bit 7:  Message appended after last packet data reply.
        _head_status = bitstring.pack("uint:8", int(row[3]))
        rospy.logdebug("Head status byte is %s", _head_status)
        if _head_status[-1]:
            rospy.logerr("Head power loss detected")
        if _head_status[-2]:
            rospy.logerr("Motor lost sync")
            self.set(force=True)

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
        hd_ctrl = bitstring.pack("uintle:16", int(row[4]))
        hd_ctrl.byteswap()  # Little endian please.
        self.inverted, self.scanright, self.continuous, self.adc8on = (
            hd_ctrl.unpack("pad:12, bool, bool, bool, bool")
        )
        rospy.logdebug("Head control bytes are %s", hd_ctrl.bin)
        rospy.logdebug("ADC8 mode %s", self.adc8on)
        rospy.logdebug("Continuous mode %s", self.continuous)
        rospy.logdebug("Scanning right %s", self.scanright)

        # Decode data settings.
        MAX_SIZE = 255 if self.adc8on else 15
        self.range = float(row[5]) / 10
        self.gain = float(row[6]) / 210
        self.ad_low = int(row[8]) * 80.0 / MAX_SIZE
        ad_span = int(row[9]) * 80.0 / MAX_SIZE
        self.ad_high = self.ad_low + ad_span

        # Scan data.
        self.nbins = int(row[14])
        self.bins = map(int, row[15:])

        # Generate configuration.
        self.config = {
            key: self.__getattribute__(key)
            for key in (
                "inverted", "continuous", "scanright",
                "adc8on", "gain", "ad_low", "ad_high",
                "left_limit", "right_limit",
                "range", "nbins", "step"
            )
        }

    def __str__(self):
        """Returns string representation of Slice."""
        return str(self.heading)


def parse(path, frame):
    """Parses scan logs and publishes LaserScan messages at set frequency.

    This publishes on two topics:
        ~heading: Pose of latest scan slice heading.
        ~scan: Point cloud of the latest scan slice.

    Args:
        path: Path to CSV log.
        frame: Name of sensor frame.
    """
    # Create publishers.
    scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)
    conf_pub = rospy.Publisher("~config", TritechMicronConfig, queue_size=800)

    with open(path) as data:
        # Read data and ignore header.
        info = csv.reader(data)
        next(info)

        previous = None
        for row in info:
            # Break cleanly if requested.
            if rospy.is_shutdown():
                break

            # Parse row.
            scan_slice = Slice(row)

            # Publish configuration as TritechMicronConfig.
            config = to_config(scan_slice.config, frame)
            conf_pub.publish(config)

            # Publish heading as PoseStamped.
            pose = to_posestamped(scan_slice.heading, frame)
            heading_pub.publish(pose)

            # Publish data as PointCloud.
            cloud = to_pointcloud(
                scan_slice.range, scan_slice.heading,
                scan_slice.bins, frame
            )
            scan_pub.publish(cloud)

            # Sleep to publish at correct rate.
            if previous:
                dt = scan_slice.timestamp - previous.timestamp
                rospy.sleep(dt.total_seconds())

            previous = scan_slice


if __name__ == "__main__":
    # Start node.
    rospy.init_node("tritech_micron")

    # Get parameters.
    path = rospy.get_param("~csv", None)
    frame = rospy.get_param("~frame", "odom")

    if path is None:
        rospy.logfatal("Please specify a file as _csv:=path/to/file.")
        sys.exit(-1)

    try:
        parse(path, frame)
    except IOError:
        rospy.logfatal("Could not find file specified.")
    except rospy.ROSInterruptException:
        pass
