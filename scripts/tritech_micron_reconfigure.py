#!/usr/bin/env python

# https://github.com/mcgill-robotics-archive/ros-tritech-micron
# http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient

import copy
import csv
import math
import tf.transformations as trans
from transforms3d.euler import quat2euler

import rclpy
# import dynamic_reconfigure.client
from rclpy.node import Node

from rospy_message_converter import message_converter

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from tritech_micron.msg import TritechMicronConfig


def is_new_scan(continuous, heading, prev_heading, prev_delta):
    is_new = False
    if not continuous:
        delta = heading - prev_heading
        if delta * prev_delta < 0:
            is_new = True
    return is_new

class TritechMicronTest(Node):

    def __init__(self):

        super().__init__('tritech_micron_test')

        self.sweeps_per_config = 2

        self.testing = False

        self.sweep_counter = 0

        self.create_subscription(PoseStamped, "heading", self.heading_callback, 10)
        self.create_subscription(TritechMicronConfig, "config", self.config_callback, 10)
        self.create_service(Empty, 'start', self.start_callback)
        # http://docs.ros.org/en/melodic/api/dynamic_reconfigure/html/dynamic_reconfigure.client.Client-class.html
        self.reconf_client = dynamic_reconfigure.client.Client("tritech_micron", timeout=10, config_callback=self.reconf_callback)

        self.declare_parameter('csv', '')
        self.csv_path = self.get_parameter('csv').get_parameter_value().string_value

        self.csv_file = None
        self.prev_heading = None
        self.prev_delta = None

    def save_current_config(self):
        self.config_saved = self.config

    def restore_config(self):
        self.reconf_client.update_configuration(self.config_saved)
        self.config_save = None
    
    def send_next_config(self):
        try:
            row = self.reader.__next__()
            self.get_logger().info(f"row: {row}")
            # config = copy.deepcopy(self.config_saved)
            config = message_converter.convert_ros_message_to_dictionary(self.config_saved)
            config.pop("header")
            config["continuous"] = False
            for key, value in row.items():
                if value != "":
                    config[key] = value
            self.get_logger().info(f"config: {config}")
            self.reconf_client.update_configuration(config)
        except StopIteration:
            self.testing = False
            # self.restore_config()
            self.get_logger().info("Testing finished")

    def start_callback(self, data):
        # with open(self.csv_path,"rb") as csv_file:
        self.csv_file = open(self.csv_path,"r")    
        self.reader = csv.DictReader(self.csv_file) # return dict
        self.csv_header = self.reader.__next__() # row 0 = header
        self.prev_heading = None
        self.save_current_config()
        self.testing = True
        self.get_logger().info("Testing started")

    def config_callback(self, data):
        self.config = data
    
    def heading_callback(self, data):
        if not self.testing:
            return
        q = data.pose.orientation
        heading = quat2euler([q.x, q.y, q.z, q.w])[2]
        # heading = (heading - math.pi) % (2*math.pi)
        if self.prev_heading is None:
            self.prev_heading = heading
            return
        if self.prev_delta is None:
            self.prev_delta = heading - self.prev_heading
            return
        self.get_logger().info(f"heading, self.prev_heading, self.prev_delta: {heading}, {self.prev_heading}, {self.prev_delta}")
        if is_new_scan(False, heading, self.prev_heading, self.prev_delta):            
            self.sweep_counter += 1
            if self.sweep_counter % self.sweeps_per_config == 0:
                self.get_logger().info("Next config")
                self.send_next_config()
        self.prev_delta = heading - self.prev_heading
        self.prev_heading = heading


    def reconf_callback(self, config):
        # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))
        pass

def main(args=None):
    rclpy.init(args=args)

    node = TritechMicronTest()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
