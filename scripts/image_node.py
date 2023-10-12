#! /usr/bin/env python

import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
# from jsk_recognition_msgs.msg import PolygonArray
from tritech_micron.msg import TritechMicronConfig

from visualize import pointcloud_to_image

class SonarMergedImage(Node):
    """
    """

    def __init__(self):

        super().__init__('sonar_merged_image')

        self.bridge = CvBridge()

        self.polyarray_msg = None

        self.create_subscription(PointCloud2, "merged_cloud", self.pc_callback, 10)
        # self.create_subscription(PolygonArray, "/polygons", self.poly_callback, 10)
        self.create_subscription(TritechMicronConfig, "config", self._config_callback, 10)

        self.image_pub = self.create_publisher(Image, "merged_image", 10)

        self.limit = 10

    # def poly_callback(self, msg):
    #     self.polyarray_msg = msg

    def _config_callback(self, msg):
        # self.config = msg
        self.limit = msg.range

    def pc_callback(self, msg):

        x, y, z = pointcloud_to_image(msg)

        # fig = plt.figure(figsize=(8, 8))
        fig, ax = plt.subplots()
        plt.axis('equal')
        ax.set_xlim(-self.limit, 0)
        ax.set_ylim(-self.limit/np.sqrt(2), self.limit/np.sqrt(2))
        
        plt.pcolormesh(x, y, z, shading='auto')

        # draw gt polygons
        # https://matplotlib.org/stable/api/_as_gen/matplotlib.patches.Polygon.html

        # if self.polyarray_msg is not None:
        #     for poly_msg in self.polyarray_msg.polygons:
        #         points = np.array([[p.x, p.y, p.z] for p in poly_msg.polygon.points])[:, :2] # ros_numpy ?
        #         p = Polygon(points, color="red", fill = False, lw=2, label="gt")
        #         ax.add_patch(p)

        # matplotlib -> OpenCV

        # https://stackoverflow.com/questions/42603161/convert-an-image-shown-in-python-into-an-opencv-image
        fig.canvas.draw()
        cv_image = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()), cv2.COLOR_RGBA2GRAY)

        # try:
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8") # "passthrough"
        # except CvBridgeError as e:
        #     print(e)

        self.image_pub.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)

    node = SonarMergedImage()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()