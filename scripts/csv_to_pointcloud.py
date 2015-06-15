#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron CSV to LaserScan and PointCloud."""

import csv
import sys
import rospy
from datetime import datetime
from collections import namedtuple
from sensor_msgs.msg import PointCloud
from tritech_micron import TritechMicron
from geometry_msgs.msg import PoseStamped
from scan import to_pointcloud, to_posestamped

__author__ = "Anass Al-Wohoush, Max Krogius"


class Slice(object):

    """Scan slice."""

    def __init__(self, row):
        """Constructs Slice object.

        Args:
            row: Current row as column array from CSV log.
            min_distance: Minimum distance in meters.
            min_intensity: Minimum intensity.
        """
        # Extract the fields in order.
        self.sof = str(row[0])
        self.timestamp = datetime.strptime(row[1], "%H:%M:%S.%f")
        self.node = int(row[2])
        self.status = int(row[3])
        self.hdctrl = int(row[4])
        self.range = float(row[5]) / 10
        self.gain = int(row[6])
        self.slope = int(row[7])
        self.ad_low = int(row[8])
        self.ad_high = self.ad_low + int(row[9])
        self.left_limit = int(row[10])
        self.right_limit = int(row[11])
        self.step = int(row[12])
        self.heading = TritechMicron.to_radians(int(row[13]))
        self.nbins = int(row[14])
        self.bins = map(int, row[15:])

        # Direction is encoded as the third bit in the HDCtrl bytes.
        self.clockwise = self.hdctrl & 0b100 > 0

    def __str__(self):
        """Returns string representation of Slice."""
        return str(self.heading)


def get_parameters():
    """Gets relevant ROS parameters into a named tuple.

    Relevant properties are:
        ~csv: Path to CSV log.
        ~rate: Publishing rate in Hz.
        ~frame: Name of sensor frame.

    Returns:
        Named tuple with the following properties:
            path: Path to CSV log.
            rate: Publishing rate in Hz.
            frame: Name of sensor frame.
    """
    options = namedtuple("Parameters", [
        "path", "rate", "frame", "width"
    ])

    options.path = rospy.get_param("~csv", None)
    options.rate = rospy.get_param("~rate", 30)
    options.frame = rospy.get_param("~frame", "odom")

    return options


def main(path, rate, frame):
    """Parses scan logs and publishes LaserScan messages at set frequency.

    This publishes on two topics:
        ~heading: Pose of latest scan slice heading.
        ~scan: Point cloud of the latest scan slice.

    Args:
        path: Path to CSV log.
        rate: Publishing rate in Hz.
        frame: Name of sensor frame.
    """
    # Create publisher.
    scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)

    rate = rospy.Rate(rate)  # Hz.

    with open(path) as data:
        # Read data and ignore header.
        info = csv.reader(data)
        next(info)

        for row in info:
            # Break cleanly if requested.
            if rospy.is_shutdown():
                break

            # Parse row.
            scan_slice = Slice(row)

            # Publish PoseStamped.
            pose = to_posestamped(scan_slice.heading, frame)
            heading_pub.publish(pose)

            # Publish PointCloud.
            cloud = to_pointcloud(
                scan_slice.range, scan_slice.heading,
                scan_slice.bins, frame
            )
            scan_pub.publish(cloud)

            rate.sleep()


if __name__ == "__main__":
    # Start node.
    rospy.init_node("tritech_micron")

    # Get parameters.
    options = get_parameters()
    if options.path is None:
        rospy.logfatal("Please specify a file as _csv:=path/to/file.")
        sys.exit(-1)

    try:
        main(options.path, options.rate, options.frame)
    except IOError:
        rospy.logfatal("Could not find file specified.")
    except rospy.ROSInterruptException:
        pass
