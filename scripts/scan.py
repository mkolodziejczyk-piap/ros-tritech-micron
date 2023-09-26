#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tritech Micron sonar scanner.

This publishes one PointCloud message per scan slice. In order to visualize in
rviz, play with the 'Decay Time' parameter. This node also provides parameters
that can be dynamically reconfigured.
"""

# import rospy
import rclpy
# from sensor_msgs.msg import PointCloud
# from sensor_msgs.msg import PointCloud2
from tritech_micron.sonar import TritechMicron
# from geometry_msgs.msg import PoseStamped
# from tritech_micron.cfg import ScanConfig
# from dynamic_reconfigure.server import Server
# from tritech_micron.msg import TritechMicronConfig

__author__ = "Anass Al-Wohoush"

def main(args=None):
    rclpy.init(args=args)

    with TritechMicron() as node:
        try:
            node.scan()
        except KeyboardInterrupt:
            node.preempt()
            node.destroy_node()

    # node = TestParams()
    # rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    # Initialize node and publishers.
    # rospy.init_node("tritech_micron")
    # scan_pub = rospy.Publisher("~scan", PointCloud, queue_size=800)
    # # scan_pub = rospy.Publisher("~scan", PointCloud2, queue_size=800)
    # heading_pub = rospy.Publisher("~heading", PoseStamped, queue_size=800)
    # conf_pub = rospy.Publisher("~config", TritechMicronConfig, queue_size=800)

    # # Get frame name and port.
    # frame = rospy.get_param("~frame")
    # port = rospy.get_param("~port")

    main()