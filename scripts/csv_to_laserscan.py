#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron CSV to LaserScan."""

import csv
import sys
import rospy
import numpy as np
from datetime import datetime
from collections import namedtuple
from sensor_msgs.msg import LaserScan

__author__ = "Anass Al-Wohoush"
__version__ = "0.0.1"


def sonar_angle_to_rad(angle):
    """Converts angles in units of 1/16th of a gradian to radians.

    Args:
        angle: Angle in 1/16th of a gradian.

    Returns:
        ANgle in radians.
    """
    return float(angle) * np.pi / 3200


class Slice(object):

    """Scan slice."""

    def __init__(self, row, min_distance, min_intensity):
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
        self.ad_span = int(row[9])
        self.left_limit = int(row[10])
        self.right_limit = int(row[11])
        self.steps = int(row[12])
        self.heading = int(row[13])
        self.num_bins = int(row[14])
        self.data = map(int, row[15:])

        # Direction is encoded as the third bit in the HDCtrl bytes.
        self.clockwise = self.hdctrl & 0b100 > 0

        # Determine range of maximum intensity.
        min_range = int(min_distance / self.range * self.num_bins)
        data = [
            intensity
            if intensity > min_intensity and index > min_range
            else 0
            for index, intensity in enumerate(self.data)
        ]
        argmax = np.argmax(data)
        self.max = argmax * self.range / self.num_bins
        self.max_intensity = self.data[argmax]

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
        self.clockwise = None
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
                or scan_slice.right_limit != self.right_limit
                or scan_slice.clockwise != self.clockwise):
            self.range = scan_slice.range
            self.num_bins = scan_slice.num_bins
            self.steps = scan_slice.steps
            self.left_limit = scan_slice.left_limit
            self.right_limit = scan_slice.right_limit
            self.clockwise = scan_slice.clockwise
            self.slices = [scan_slice]
            return

        # Remove oldest if full.
        if self.full():
            self.slices.pop(0)
        self.slices.append(scan_slice)

    def to_laser_scan(self, frame, queue=0):
        """Converts current scan to LaserScan message.

        Args:
            frame: Frame name.
            queue: Queue size (0 means full scan).

        Returns:
            LaserScan.
        """
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

        # Set time and time increments.
        scan.time_increment = (
            queued_slices[-1].timestamp - queued_slices[0].timestamp
        ).total_seconds() if len(queued_slices) > 1 else 0
        scan.scan_time = (
            queued_slices[-1].timestamp - queued_slices[0].timestamp
        ).total_seconds()

        # Set angular range.
        scan.angle_min = sonar_angle_to_rad(0)
        scan.angle_max = sonar_angle_to_rad(6400)
        scan.angle_increment = sonar_angle_to_rad(self.steps)

        # Determine ranges and intensities.
        num_of_headings = 6400 / self.steps
        scan.range_max = self.range
        scan.range_min = 0.1
        scan.ranges = [0 for i in range(num_of_headings)]
        scan.intensities = [0 for i in range(num_of_headings)]
        for scan_slice in queued_slices:
            index = scan_slice.heading / self.steps
            scan.ranges[index] = scan_slice.max
            scan.intensities[index] = scan_slice.max_intensity

        return scan


def get_parameters():
    """Gets relevant ROS parameters into a named tuple.

    Relevant properties are:
        ~csv: Path to CSV log.
        ~queue: Queue for sector scan message.
        ~rate: Publishing rate in Hz.
        ~frame: Frame name.
        ~min_distance: Minimum distance in meters.
        ~min_intensity: Minimum intensity.

    Returns:
        Named tuple with the following properties:
            path: Path to CSV log.
            queue: Queue for sector scan message.
            rate: Publishing rate in Hz.
            frame: Frame name.
            min_distance: Minimum distance in meters.
            min_intensity: Minimum intensity.
    """
    options = namedtuple("Parameters", [
        "path", "queue", "rate", "frame", "min_distance", "min_intensity"
    ])

    options.path = rospy.get_param("~csv", None)
    options.queue = rospy.get_param("~queue", 10)
    options.rate = rospy.get_param("~rate", 30)
    options.frame = rospy.get_param("~frame", "odom")
    options.min_distance = rospy.get_param("~min_distance", 1)
    options.min_intensity = rospy.get_param("~min_intensity", 50)

    return options


def main(path, queue, rate, frame, min_distance, min_intensity):
    """Parses scan logs and publishes LaserScan messages at set frequency.

    This publishes on two topics:
        /sonar/full: A full 360 degree scan.
        /sonar/sector: A queued scan with the past few headings.

    Args:
        path: Path to CSV log.
        queue: Queue for sector scan message.
        rate: Publishing rate in Hz.
        frame: Frame name.
        min_distance: Minimum distance in meters.
        min_intensity: Minimum intensity.
    """
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
            # Break cleanly if requested.
            if rospy.is_shutdown():
                break

            # Parse row.
            scan_slice = Slice(row, min_distance, min_intensity)
            scan.add(scan_slice)

            # Publish full scan.
            laser_scan = scan.to_laser_scan(frame)
            if laser_scan:
                full_pub.publish(laser_scan)

            # Publish sector scan.
            laser_scan = scan.to_laser_scan(frame, queue=queue)
            if laser_scan:
                sector_pub.publish(laser_scan)

            rate.sleep()


if __name__ == "__main__":
    # Start node.
    rospy.init_node("sonar")

    # Get parameters.
    options = get_parameters()
    if options.path is None:
        rospy.logfatal("Please specify a file.")
        sys.exit(-1)

    try:
        main(
            options.path, options.queue, options.rate, options.frame,
            options.min_distance, options.min_intensity
        )
    except IOError:
        rospy.logfatal("Could not find file specified.")
    except rospy.ROSInterruptException:
        pass
