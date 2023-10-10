#! /usr/bin/env python3

import copy
import time
import threading

import numpy as np
# import tf.transformations as trans

import rclpy
from rclpy.node import Node

import tf2_ros
# from ros_numpy import msgify, numpify
# from tf.transformations import euler_from_matrix
from transforms3d.euler import mat2euler, quat2euler

from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, PointCloud2
# from sensor_msgs.point_cloud2 import create_cloud
from tritech_micron.msg import TritechMicronConfig
from std_msgs.msg import Float32MultiArray

# https://github.com/ros/geometry2/blob/382e5ca5ff5395e68cd39b37b3a7527a739b4843/tf2_sensor_msgs/src/tf2_sensor_msgs/tf2_sensor_msgs.py#L52-L59
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

# https://github.com/ctu-vras/cloud_proc/blob/master/scripts/odom_cloud_transformer

def pointcloud_to_image(pc: PointCloud2, resolution=0.5):
    """
    Converts the 2D pointcloud to image. Each image pixel comes from interpolation
    """

    # PointCloud2 to numpy

    pc_np = numpify(pc)

    # min/max

    xy_min = np.min(pc_np[:, :2])
    xy_max = np.max(pc_np[:, :2])

    # np.array

    import scipy.interpolate

    # z_all = plotGrid.astype(float)            # convert nones to nan
    # x_all, y_all = np.indices(plotGrid.shape) # get x and y coordinates

    x_all = np.arange(xy_min[0], xy_max[0], resolution)
    y_all = np.arange(xy_min[1], xy_max[1], resolution)

    # convert to 1d arrays of coordinates
    valid = ~np.isnan(z_all)
    x, y, z = x_all[valid], y_all[valid], z_all[valid]

    # interpolate
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp2d.html
    # jax: https://github.com/google/jax/discussions/10689
    interp = scipy.interpolate.interp2d(x, y, z)
    filled_data = interp(x_all[:,0], y_all[0,:])  # this is kinda gross, but `interp` doesn't
                                                # do normal broadcasting

    return filled_data


def to_pc_msg(pc_ref_msg, width, data):
    print("to_pc_msg 1")
    pc_msg = copy.deepcopy(pc_ref_msg)
    pc_msg.width = int(width)
    # pc_msg.data = data.astype(np.int8).flatten().tobytes()
    pc_msg.data = data.astype(np.float32).flatten().tobytes()    
    print("to_pc_msg 2")
    return pc_msg

def merge_pcs(pcs):
    """
    First pointcloud is copied and fields width and data are updated
    Args:
        pcs: pointclouds to be merged
    Returns:
        Merged pointcloud
    """

    tic = time.time()
    # pc = copy.deepcopy(pcs[0])
    # pc.width = np.sum(np.array([p.width for p in pcs]))        
    # data_np = np.concatenate([np.frombuffer(p.data, dtype=np.uint8) for p in pcs])
    # pc = copy.deepcopy(pc_msg)
    data_np = np.concatenate([p for p in pcs])
    width = np.sum([p.shape[0] for p in pcs])     
    # data_bytes = data_np.tobytes()
    # pc.data = data_bytes
    # print(f"len(pcs): {len(pcs)}")
    print(f"{(time.time() - tic):0.3f} sec")   

    # for pc in self.pcs:
        # self.cloud_pub.publish(pc)

    # read_points()
    # create_cloud(header, fields, points)

    # return pc
    return data_np, width

# def do_transform_cloud_jax(pc, R, T):
def do_transform_cloud_np(pc_np, M):
    """
    pc: (n, 3)
    R: (3, 3)
    T: (3, 1)
    """

    R = M[:3,:3]
    T = M[:3,3]  

    pc_tf = T.T + pc_np[:, :3] @ R.T
    pc_tf = np.concatenate((pc_tf, pc_np[:,[3]]), axis=1)

    return pc_tf

def pc_msg_to_np(pc_msg):

    # pc_np = numpify(pc_msg)
    # pc_np = np.vstack([points[f] for f in ['x', 'y', 'z']])  
    # pc_np = np.reshape(np.frombuffer(pc_msg.data, dtype=np.uint8), (-1,4))
    pc_np = np.reshape(np.frombuffer(pc_msg.data, dtype=np.float32), (-1,4))

    return pc_np


# def transform_pc_by_matrix(pc_msg, M):
#     """
#     First pointcloud is copied and fields width and data are updated
#     Args:
#         pc: pointcloud
#         M: transform matrix
#     Returns:
#         transformaed pointcloud
#     """
#     tfs_msg = TransformStamped()
#     tfs_msg.header = pc_msg.header
#     tfs_msg.child_frame_id = "" # child_frame_id is not used in do_transform_cloud()
#     tfs_msg.transform = msgify(Transform, M)

#     tic = time.time()     
#     # pc = do_transform_cloud(pc_msg, tfs_msg)
#     pc = do_transform_cloud_jax(pc_msg, tfs_msg)
#     print(f"do_transform_cloud(pc_msg, tfs_msg): {(time.time() - tic):0.3f} sec")
#     # pc.header.frame_id = pc_msg.header.frame_id

#     return pc

def relative_matrix(M0, M1):

    M0_inv = np.linalg.inv(M0)        
    M01 = np.dot(M0_inv, M1)
    
    return M01



class TritechMicronMerge(Node):
    """


    """

    def __init__(self):

        super().__init__('tritech_micron_merge')

        self.debug = False

        # self.declare_parameter('namespace', 'lauv')
        # self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.namespace = "lauv"

        # self.declare_parameter('frame_id', f"{self.namespace}/tritech_micron_link")
        # self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.frame_id = f"{self.namespace}/tritech_micron_link"
        
        self.direction = 1
        self.pos = 3.0
        self.M0 = None
        
        self.current_pcs = []
        self.done_pcs = None

        self.current_headers = []
        self.done_headers = None

        self.pc_ref_msg = None

        self.config = None

        # world -> base_link -> sonar_link -> beam_link

        # for world -> sonar_link position, alternatively a topic with world -> base_link and static tf base_link -> sonar_link
        # self.tfBuffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # self.last_update = rospy.Time.now() 

        # subscribtion

        self.create_subscription(PointCloud2, "scan", self.pc_callback, 10)
        # ros_tritech_micron
        self.create_subscription(PoseStamped, 'heading', self.heading_callback, 10)

        # base_link pose from localization
        # self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)

        # gazebo
        self.create_subscription(JointState, 'joint_states', self._joint_state_callback, 10)

        self.create_subscription(TritechMicronConfig, "config", self._config_callback, 10)

        # publishers

        self.cloud_pub = self.create_publisher(PointCloud2, "merged_cloud", 10)

        # self.image_pub = rospy.Publisher("merged_image", Image, queue_size=10)

        # config for merged cloud
        self.config_pub = self.create_publisher(TritechMicronConfig, "merged_config", 10)

        self.stats_pub = self.create_publisher(Float32MultiArray, "merged_stats", 10)

        # https://www.bogotobogo.com/python/Multithread/python_multithreading_Event_Objects_between_Threads.php

        self.lock = threading.Lock()
        
        # https://superfastpython.com/thread-event-object-in-python/
        self.event = threading.Event()
        self.t = threading.Thread(name='cloud_assembly', 
                      target=self.cloud_assembly,
                      args=(self.event,)
                    )

        self.t.start()

        # # or tf

        

        # self.start_pose = None
        # self.heading = None

        # self.raw_data = [] # (tf odom->sonar, pc, heading?)
        # self.transformed_pcs = []

    def cloud_assembly(self, event):

        while True:
            event.wait()
            print(f"wait event")
            if self.config is None:
                config = TritechMicronConfig()
            else:
                config = copy.deepcopy(self.config)
            fixed_frame = f"{self.namespace}/tritech_micron_link_fixed"
            M0 = self.M0
            M1 = self.current_sonar_matrix(fixed_frame)
            # transform forward to current frame
            M10 = relative_matrix(M1, M0)
            # pc = merge_pcs(self.done_pcs)
            tic = time.time()
            data_np, width = merge_pcs(self.done_pcs)            
            # pc_np = transform_pc_by_matrix(pc, M10)
            data_np = do_transform_cloud_np(data_np, M10)
            print(f"data_np.shape {data_np.shape}")
            print(f"data_np.dtype {data_np.dtype}")
            print(f"merge: {(time.time() - tic):0.3f} sec")
            pc = to_pc_msg(self.pc_ref_msg, width, data_np)
            # update stamp
            stamp = self.get_clock().now().to_msg()
            pc.header.stamp = stamp
            pc.header.frame_id = fixed_frame
            print("PUBLISH")                              
            self.cloud_pub.publish(pc)
            print(f"published")
            config.header.stamp = stamp
            self.config_pub.publish(config)

            # sweep time
            # number of scans

            sweep_time = 0.0 # self.done_headers[-1].stamp.seconds - self.done_headers[0].stamp.seconds
            n_scans = float(len(self.done_headers))

            stats_msg = Float32MultiArray()
            stats_msg.data = [
                sweep_time,
                n_scans
            ]
            self.stats_pub.publish(stats_msg)

            event.clear()

    def current_sonar_matrix(self, target_frame=None):

        # if target_frame is None:
        #     target_frame=self.frame_id
        
        # try:
        #     tf_msg = self.tfBuffer.lookup_transform("world",
        #                                             target_frame,
        #                                             rospy.Time.now(),rospy.Duration(1.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logwarn("Error trying to look for transform ")

        # M = numpify(tf_msg.transform)
        M = np.identity(4)

        return M
    
    def heading_callback(self, msg):

        # PoseStamped to JointState
        js_msg = JointState()
        js_msg.name = [f"{self.namespace}/multibeam_sonar_joint"]
        q = msg.pose.orientation
        heading = quat2euler([q.w, q.x, q.y, q.z])[2]
        print(f"heading: {heading}")
        js_msg.position = [heading]

        self._joint_state_callback(js_msg)
    
    def _joint_state_callback(self, msg):

        id = msg.name.index(f"{self.namespace}/multibeam_sonar_joint")
        # pos = msg.position[id] % (2*np.pi)
        # pos = (msg.position[id]+np.pi/8) % (2*np.pi)
        pos = msg.position[id]

        # print(f"velocity: {msg.velocity[id]}")

        # print(pos)
        # if pos < np.pi and self.pos > np.pi:
        diff = round(pos-self.pos,4)
        print(f"diff 1: {diff}")
        diff = self.direction * diff
        print(f"diff 2: {diff}")
        if diff < 0:
            #TODO lock self.pcs
            # print(len(self.pcs))
            print(f"self.direction: {self.direction}")
            print(f"pos-self.pos: {pos-self.pos}")
            print("merge")
            with self.lock:
                self.done_pcs = self.current_pcs                
                self.current_pcs = []
                self.done_headers = self.current_headers                
                self.current_headers = []
            print(f"len(self.done_pcs): {len(self.done_pcs)}")
            if len(self.done_pcs) > 0:
                # print(self.pcs)

                # pc = merge_pcs(self.pcs)
                print(f"set event")
                self.event.set() # trigger merge of pointclouds
                # for pc in self.pcs:
                    # self.cloud_pub.publish(pc)
            self.M0 = self.current_sonar_matrix()
            # self.pcs = []
            self.direction *= -1
        
        self.pos = pos

    def _config_callback(self, msg):
        self.config = msg

    def pc_callback(self, msg):

        # frame_id = msg.header.frame_id

        print("step 1")

        if self.pc_ref_msg is None:
            self.pc_ref_msg = msg

        #TODO lock M0
        if self.M0 is None:
            return
        else:
            M0 = self.M0

        print("step 2")
        M1 = self.current_sonar_matrix()

        # euler = euler_from_matrix(M1)
        # print(euler)

        # M1_inv = np.linalg.inv(M1)        
        # M10 = np.dot(M1_inv, M0)
        # euler = euler_from_matrix(M10)
        # print(euler)
   
        M01 = relative_matrix(M0, M1)
        
        if self.debug:
            euler = mat2euler(M01)
            print(euler)


        print("step 3")

        # pc = transform_pc_by_matrix(msg, M01)
        pc_np = pc_msg_to_np(msg)
        print(f"pc_np.dtype {pc_np.dtype}")
        pc = do_transform_cloud_np(pc_np, M01)

        with self.lock:
            # append, extend? other structure?
            self.current_pcs.append(pc)
            self.current_headers.append(msg.header)

        print("step 4")

def main(args=None):
    rclpy.init(args=args)

    node = TritechMicronMerge()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
