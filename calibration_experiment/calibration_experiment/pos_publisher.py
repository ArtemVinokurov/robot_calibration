#!/usr/bin/env python3

import rclpy.lifecycle
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
from robot_interfaces.srv import MoveJ, TrackerPose
from std_msgs.msg import Float32
import rclpy.time

import motorcortex
import math
import time
from ament_index_python.packages import get_package_share_directory
import os
import csv
import json

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Transform
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R

from pathlib import Path
from std_srvs.srv import Empty
import matplotlib.pyplot as plt

from std_msgs.msg import Float32

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Pose, Vector3

from builtin_interfaces.msg import Duration
import sys

class PosPublisher(Node):
    def __init__(self):
        super().__init__('pos_publisher')
        self.x_publisher_ = self.create_publisher(Float32, 'pos_x', 10)
        self.y_publisher_ = self.create_publisher(Float32, 'pos_y', 10)
        self.z_publisher_ = self.create_publisher(Float32, 'pos_z', 10)

        self.marker_publisher_ = self.create_publisher(Marker, 'tracker_marker', 1000)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.target_frame = 'tracker_LHR_A5314E9D'
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

        self.ax.set_xlabel('X Label')
        self.ax.set_ylabel('Y Label')
        self.ax.set_zlabel('Z Label')

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.id = 0

    def timer_callback(self):
        from_frame = 'world_vive'
        try:
            t = self.tf_buffer.lookup_transform(from_frame, self.target_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.target_frame} to {from_frame}: {ex}')
            return
    
        self.ax.scatter(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, marker="x")


        marker = Marker()
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        marker.pose = pose
        
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'world_vive'

        duration = Duration()
        duration.sec = 0
        # marker.lifetime = duration

        vec = Vector3()
        vec.x = 0.001
        vec.y = 0.001
        vec.z = 0.001
        marker.scale = vec
        marker.action = Marker.MODIFY
        marker.type = Marker.SPHERE
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.id = self.id

        self.marker_publisher_.publish(marker)
        self.id += 1

def main():
    rclpy.init()
    node = PosPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
