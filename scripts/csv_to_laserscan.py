#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron CSV to LaserScan."""

import csv
import sys
import rospy
import numpy as np
from datetime import datetime
from sensor_msgs.msg import LaserScan

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


def sonar_angle_to_rad(angle):
    """Converts angles in units of 1/16th of a gradian to radians."""
    return float(angle) * np.pi / 3200


class Slice(object):

    """Scan slice."""

    def __init__(self, *args):
        """Constructs Slice object.

        Args:
            *args: Columns from CSV log.
        """
        self.sof = str(args[0])
        self.timestamp = datetime.strptime(args[1], "%H:%M:%S.%f")
        self.node = int(args[2])
        self.status = int(args[3])
        self.hdctrl = int(args[4])
        self.range = float(args[5]) / 10
        self.gain = int(args[6])
        self.slope = int(args[7])
        self.ad_low = int(args[8])
        self.ad_span = int(args[9])
        self.left_limit = int(args[10])
        self.right_limit = int(args[11])
        self.steps = int(args[12])
        self.heading = int(args[13])
        self.num_bins = int(args[14])
        self.data = map(int, args[15:])
        argmax = np.argmax(self.data[60:]) + 60
        self.max = argmax * self.range / self.num_bins
        self.max_intensity = self.data[argmax]

        # Direction is encoded as the third bit in the HDCtrl bytes.
        self.scanright = self.hdctrl & 0b100 > 0

    def __str__(self):
        """Returns string representation of Slice."""
        return str(self.heading)


class Scan(object):

    """Scan."""

    def __init__(self):
        """Constructs Scan object."""
        self.range = None
        self.steps = None
        self.num_bins = None
        self.left_limit = None
        self.right_limit = None
        self.previous_direction = None
        self.slices = []

    def empty(self):
        """Returns whether the scan is empty."""
        return len(self.slices) == 0

    def full(self):
        """Returns whether the scan contains a full scan of data."""
        if self.steps is None:
            return False

        # Continuous scan or sector scan.
        if self.right_limit == 3201 and self.left_limit == 3199:
            max_range = 6400
        else:
            max_range = (self.right_limit - self.left_limit) % 6400

        return len(self.slices) == (max_range / self.steps)

    def add(self, scan_slice):
        """Adds a slice to the scan.

        Args:
            scan_slice: Slice of the scan.
        """
        # Reset if slice is of different scan.
        if (scan_slice.range != self.range
                or scan_slice.num_bins != self.num_bins
                or scan_slice.steps != self.steps
                or scan_slice.left_limit != self.left_limit
                or scan_slice.right_limit != self.right_limit):
            self.range = scan_slice.range
            self.num_bins = scan_slice.num_bins
            self.steps = scan_slice.steps
            self.left_limit = scan_slice.left_limit
            self.right_limit = scan_slice.right_limit
            self.slices = [scan_slice]
            return

        # Reset if direction is switched.
        if self.slices[-1].scanright != scan_slice.scanright:
            self.slices = [scan_slice]
            return

        # Remove oldest if full.
        if self.full():
            self.slices.pop(0)
        self.slices.append(scan_slice)

    def to_laser_scan(self, queue=0, frame="odom"):
        # Return nothing if empty.
        if len(self.slices) < queue or self.empty():
            return None

        scan = LaserScan()

        # Header.
        scan.header.frame_id = frame
        scan.header.stamp = rospy.get_rostime()

        # Get latest N slices.
        queued_slices = sorted(
            self.slices, key=lambda x: x.timestamp
        )[-1 * queue:]
        if not self.slices[0].scanright:
            queued_slices.reverse()

        # Set time and time increments.
        scan.time_increment = (
            queued_slices[-1].timestamp - queued_slices[0].timestamp
        ).total_seconds() if len(queued_slices) > 1 else 0
        scan.scan_time = (
            queued_slices[-1].timestamp - queued_slices[0].timestamp
        ).total_seconds()

        # Determine angular range.
        angle_min = queued_slices[0].heading
        angle_max = queued_slices[-1].heading
        if angle_max < angle_min:
            angle_max += 6400
        scan.angle_min = sonar_angle_to_rad(angle_min)
        scan.angle_max = sonar_angle_to_rad(angle_max)
        scan.angle_increment = sonar_angle_to_rad(self.steps)

        # Set ranges and intensities.
        scan.range_max = self.range
        scan.range_min = 0.1
        scan.ranges = [
            scan_slice.max
            for scan_slice in queued_slices
        ]
        scan.intensities = [
            scan_slice.max_intensity
            for scan_slice in queued_slices
        ]

        return scan


def main(path, queue, rate):
    # Create publisher.
    full_pub = rospy.Publisher("sonar/full", LaserScan, queue_size=10)
    sector_pub = rospy.Publisher("sonar/sector", LaserScan, queue_size=10)

    rate = rospy.Rate(rate)  # Hz

    scan = Scan()
    with open(path) as data:
        # Read data and ignore header.
        info = csv.reader(data)
        next(info)

        for row in info:
            scan_slice = Slice(*row)
            scan.add(scan_slice)

            # Publish full scan.
            laser_scan = scan.to_laser_scan(queue=0)
            if laser_scan:
                full_pub.publish(laser_scan)

            # Publish sector scan.
            laser_scan = scan.to_laser_scan(queue=queue)
            if laser_scan:
                sector_pub.publish(laser_scan)

            rate.sleep()


if __name__ == "__main__":
    # Start node.
    rospy.init_node("sonar")

    # Get path to CSV.
    if rospy.has_param("~csv"):
        path = rospy.get_param("~csv")
    else:
        rospy.logfatal("Please specify path to CSV")
        sys.exit(-1)

    # Get queue size.
    if rospy.has_param("~queue"):
        queue = rospy.get_param("~queue")
    else:
        queue = 10

    # Get rate.
    if rospy.has_param("~rate"):
        rate = rospy.get_param("~rate")
    else:
        rate = 30

    try:
        main(path, queue, rate)
    except IOError:
        rospy.logfatal("Could not find file specified")
    except rospy.ROSInterruptException:
        pass
