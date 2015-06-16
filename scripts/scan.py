#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Tritech Micron sonar scanner.

This publishes one PointCloud message per scan slice. In order to visualize in
rviz, play with the 'Decay Time' parameter. This node also provides parameters
that can be dynamically reconfigured.
"""

import math
import rospy
from tritech_micron import TritechMicron
from tritech_micron.cfg import ScanConfig
from dynamic_reconfigure.server import Server
from tritech_micron.msg import TritechMicronConfig
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import ChannelFloat32, PointCloud
from geometry_msgs.msg import Point32, Pose, PoseStamped, Quaternion

__author__ = "Anass Al-Wohoush"


def to_config(config, frame):
    """Converts a scan slice to a TritechMicronConfig message.

    Args:
        config: Dictionary of sonar configuration.
        frame: Frame ID.

    Returns:
        TritechMicronConfig.
    """
    config = TritechMicronConfig(**config)
    config.header.frame_id = frame
    config.header.stamp = rospy.get_rostime()
    return config


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
    rospy.logwarn("Reconfiguring sonar")
    rospy.logdebug("Configuration requested: %r, %r", config, level)

    # Remove additional keys.
    if "groups" in config:
        config.pop("groups")

    # Set parameters.
    sonar.set(**config)
    return config


def publish(sonar, range_scale, heading, bins, config):
    """Publishes PointCloud, PoseStamped and TritechMicronConfig of current
    scan slice on callback.

    Args:
        sonar: Sonar instance.
        range_scale: Current scan range in meters.
        heading: Current heading in radians.
        bins: Integer array with the intensity at every bin.
        config: Sonar configuration for current scan slice.
    """

    # Publish heading as PoseStamped.
    posestamp = to_posestamped(heading, frame)
    heading_pub.publish(posestamp)

    # Publish data as PointCloud.
    cloud = to_pointcloud(range_scale, heading, bins, frame)
    scan_pub.publish(cloud)

    # Publish data as TritechMicronConfig.
    config = to_config(config, frame)
    conf_pub.publish(config)


if __name__ == "__main__":
    # Initialize node and publishers.
    rospy.init_node("tritech_micron")
    scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)
    conf_pub = rospy.Publisher("~config", TritechMicronConfig, queue_size=800)

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
