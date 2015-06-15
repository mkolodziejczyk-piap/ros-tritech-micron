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
from tritech_micron import TritechMicron
from tritech_micron.cfg import ScanConfig
from dynamic_reconfigure.server import Server
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import ChannelFloat32, PointCloud
from geometry_msgs.msg import Point32, Pose, PoseStamped, Quaternion

__author__ = "Anass Al-Wohoush"


def to_pointcloud(range_scale, heading, bins, frame):
    """Converts a scan slice to a PointCloud message.

    Args:
        range_scale: Range of scan.
        heading: Slice heading in radians.
        bins: Array of intensities of each return.
        frame: Frame ID.

    Returns:
        A sensor_msgs.msg.PointCloud.
    """
    # Construct PointCloud message.
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


def to_posestamped(heading, frame):
    """Converts a heading to a PoseStamped message.

    Args:
        heading: Slice heading in radians.
        frame: Frame ID.

    Returns:
        A geometry_msgs.msg.PoseStamped.
    """
    # Construct PoseStamped message.
    posestamp = PoseStamped()
    posestamp.header.frame_id = frame
    posestamp.header.stamp = rospy.get_rostime()

    # Convert to quaternion.
    q = Quaternion(*quaternion_from_euler(0, 0, heading))

    # Make Pose message.
    pose = Pose(orientation=q)
    posestamp.pose = pose

    return posestamp


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


def publish(sonar, range_scale, heading, bins):
    """Publishes PointCloud and PoseStamped of current scan slice on callback.

    Args:
        sonar: Sonar instance.
        range_scale: Current scan range in meters.
        heading: Current heading in radians.
        bins: Integer array with the intensity at every bin.
    """
    # Publish range as Float64.
    range_pub.publish(range_scale)

    # Publish heading as PoseStamped.
    posestamp = to_posestamped(heading, frame)
    heading_pub.publish(posestamp, frame)

    # Publish data as PointCloud.
    cloud = to_pointcloud(range_scale, heading, bins)
    scan_pub.publish(cloud)


if __name__ == "__main__":
    # Initialize node and publishers.
    rospy.init_node("tritech_micron")
    scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    range_pub = rospy.Publisher("~range", Float64, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)

    # Get frame name and port.
    frame = rospy.get_param("~frame", "odom")
    port = rospy.get_param("~port", "/dev/sonar")

    with TritechMicron(port=port) as sonar:
        try:
            # Initialize dynamic reconfigure server and scan.
            Server(ScanConfig, reconfigure)

            # Scan.
            sonar.scan(callback=publish)
        except KeyboardInterrupt:
            sonar.preempt()
