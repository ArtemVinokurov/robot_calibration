#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
from robot_interfaces.srv import MoveJ

import rclpy.time

import motorcortex
import math
import time
from .robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
from .robot_control.robot_command import RobotCommand
from .robot_control.system_defs import InterpreterStates
from ament_index_python.packages import get_package_share_directory
import os
import csv

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Transform

import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R


from pathlib import Path


class DataCollectionNode(Node):
    def __init__(self):
        super().__init__("data_collection_node")

        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'mcx.cert.pem')

        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.57.3:5568:5567', self.motorcortex_types, parameter_tree,
                                                     certificate=license_file, timeout_ms=1000, login="admin", password="vectioneer")
        
        # self.subscription = self.sub
            self.joint_subscription = self.sub.subscribe(
                ['root/ManipulatorControl/jointPositionsActual'], 'group1', 1)
            self.tool_subscription = self.sub.subscribe('root/ManipulatorControl/manipulatorToolPoseActual', 'group2', 1)
            self.tool_subscription.get()
            self.joint_subscription.get()            
        except Exception as e:
            self.get_logger().error(f"Failed to establish connection: {e}")
            return
        
        callback_group = ReentrantCallbackGroup()

        self.cli = self.create_client(MoveJ, 'moveJ', callback_group=callback_group)

        self.tracker_pose = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = 'tracker_LHR_A5314E9D'
        self.timer = self.create_timer(0.01, self.tracker_pose_read, callback_group=callback_group)

        self.path_to_traj = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'data.csv')
        self.goal_trajectory = []

        self.path_to_csv = os.path.expandvars('$HOME') + '/experiments/data.json'

    def get_trajectory(self):

        trajectory_array = []
        with open(self.path_to_traj, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                lst = map(float, row)
                trajectory_array.append(list(lst)[:6])  
        
        return trajectory_array


    def tracker_pose_read(self):
        from_frame = 'world_vive'
        to_frame = self.target_frame

        try:
            t = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame} to {from_frame}: {ex}')
            return
    
        rotation = R.from_quat(np.array([t.transform.rotation.x,
                                         t.transform.rotation.y,
                                         t.transform.rotation.z,
                                         t.transform.rotation.w]))    
        translation = np.array([[t.transform.translation.x],
                                [t.transform.translation.y],
                                [t.transform.translation.z]])
        
        Rt = np.append(rotation, translation, axis=1)
        self.tracker_pose = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])
        
    def get_tracker_pose_array(self):
        rotation_mat = R.from_matrix(self.tracker_pose[:3, :3])

        translation_vec = self.tracker_pose[:,3]

        result = np.concatenate([translation_vec, rotation_mat.as_euler('zyx', degrees=False)])

        return result.tolist()



    def base_calibration(self):
        initial_pos = [0.0, 0.0, math.radians(90.0), 0.0, math.radians(90.0), 0.0]

        req = MoveJ.Request()
        req.angles = initial_pos
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(2.0)

        tool_params = self.tool_subscription.read()[0].value

        tool_rotation = R.from_euler('zyx', np.array(tool_params[3:]))
        tool_trans = tool_params[:3]

        Rt = np.append(tool_rotation.as_matrix(), [[tool_trans[0]], [tool_trans[1]], [tool_trans[2]]], axis=1)
        tool_tf = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])
        tracker_tf = self.tracker_pose
        base_frame_tf = tracker_tf @ inv(tool_tf)

        ### write base frame tf to json
        # 
        # 
        # 
        #  

        self.get_logger().info("Calibration of the base frame has been done")
    


    def execute_experiment(self):
        self.get_logger().info("Starting experiment...")
        
        goal_trajectory = self.get_trajectory()

        for traj in goal_trajectory:
            req = MoveJ.Request()
            req.angles = traj
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(5.0)

            folder = Path(self.path_to_csv)
            file_count = len(list(folder.iterdir()))

            header = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6',
              'px', 'py', 'pz', 'Rz', 'Ry', 'Rx']
            
            with open(self.path_to_csv + "\experiment" + str(file_count), "w", encoding='UTF8') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                actual_joint_position = self.joint_subscription.read()[0].value
                tracker_position = self.get_tracker_pose_array()
                data = actual_joint_position + tracker_position
                writer.writerow(data)

                self.get_logger().info(f"Write point with coordinates: x={tracker_position[0]}, y={tracker_position[1]}, z={tracker_position[2]}, Rz={tracker_position[3]}, Ry={tracker_position[4]}, Rx{tracker_position[5]}")
                

            time.sleep(0.5)

def main():
    rclpy.init()

    node = DataCollectionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down..')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        

        














            




















        









        
    
    def quat2Euler(self, quat):
        qw = quat[0]
        qx = quat[1]
        qy = quat[2]
        qz = quat[3]
        euler_angles = [math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)), 2 * math.atan2((1 + 2 * (qw * qy - qx * qz)) ** 0.5, (1 - 2 * (qw * qy - qx * qz)) ** 0.5) - math.pi / 2, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))]
        return euler_angles


