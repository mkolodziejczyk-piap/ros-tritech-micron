#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron sonar scanner.

This publishes one PointCloud message per scan slice. In order to visualize in
rviz, play with the 'Decay Time' parameter. This node also provides parameters
that can be dynamically reconfigured.
"""

import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point32
from tritech_micron import TritechMicron
from tritech_micron.cfg import ScanConfig
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import ChannelFloat32, PointCloud

__author__ = "Anass Al-Wohoush"


def to_pointcloud(range_scale, heading, bins, frame="odom"):
    """Converts a scan slice to a PointCloud message.

    Args:
        range_scale: Range of scan.
        heading: Slice heading in radians.
        bins: Array of intensities of each return.
        frame: Name of sensor frame.

    Returns:
        A sensor_msgs.msg.PointCloud.
    """
    # Construct PointCloud message
    cloud = PointCloud()
    cloud.header.frame_id = frame
    cloud.header.stamp = rospy.get_rostime()

    # Convert bins to list of Point32 messages.
    nbins = len(bins)
    r_step = range_scale / nbins
    x_unit = math.cos(heading) * r_step
    y_unit = math.sin(heading) * r_step
    cloud.points = [
        Point32(x=x_unit * r, y=y_unit * r, z=0.)
        for r in range(1, nbins + 1)
    ]

    # Set intensity channel.
    channel = ChannelFloat32()
    channel.name = "intensity"
    channel.values = bins
    cloud.channels = [channel]

    return cloud


def reconfigure(config, level):
    """Reconfigures sonar dynamically.

    Args:
        config: New configuration.
        level: Level bitmask.

    Returns:
        Configuration.
    """
    rospy.loginfo("Reconfiguring sonar: %r, %r", config, level)

    # Remove additional keys.
    if "groups" in config:
        config.pop("groups")

    # Set parameters.
    sonar.set(**config)
    return config


def publish(sonar, range, heading, bins):
    """Publishes PointCloud of current scan slice on callback.

    Args:
        sonar: Sonar instance.
        range: Current scan range in meters.
        heading: Current heading in radians.
        bins: Integer array with the intensity at every bin.
    """
    heading_pub.publish(heading)
    cloud = to_pointcloud(range, heading, bins)
    cloud_pub.publish(cloud)


if __name__ == "__main__":
    # Initialize node and publisher.
    rospy.init_node("tritech_micron", log_level=rospy.INFO)
    cloud_pub = rospy.Publisher(
        "/tritech_micron/scan",
        PointCloud, queue_size=800
    )
    heading_pub = rospy.Publisher(
        "/tritech_micron/heading",
        Float64, queue_size=800
    )

    with TritechMicron() as sonar:
        try:
            # Initialize dynamic reconfigure server and scan.
            Server(ScanConfig, reconfigure)

            # Scan.
            sonar.scan(feedback_callback=publish)
        except KeyboardInterrupt:
            sonar.preempt()
