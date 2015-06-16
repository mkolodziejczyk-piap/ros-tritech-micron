# -*- coding: utf-8 -*-

"""Tritech Micron sonar tools."""

import math
import rospy
from tritech_micron.msg import TritechMicronConfig
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import ChannelFloat32, PointCloud
from geometry_msgs.msg import Point32, Pose, PoseStamped, Quaternion

__author__ = "Anass Al-Wohoush"


def to_sonar_angles(rad):
    """Converts radians to units of 1/16th of a gradian.

    Args:
        rad: Angle in radians.

    Returns:
        Integral angle in units of 1/16th of a gradian.
    """
    return int(rad * 3200 / math.pi) % 6400


def to_radians(angle):
    """Converts units of 1/16th of a gradian to radians.

    Args:
        angle: Angle in units of 1/16th of a gradian.

    Returns:
        Angle in radians.
    """
    return angle / 3200.0 * math.pi


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
