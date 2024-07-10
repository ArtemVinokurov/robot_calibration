#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
from robot_interfaces.srv import MoveJ
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

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__("data_collection_node")

        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'mcx.cert.pem')

        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.2.100:5568:5567', self.motorcortex_types, parameter_tree,
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

        self.movej_cli = self.create_client(MoveJ, 'moveJ', callback_group=callback_group)
        self.set_origin_cli = self.create_client(Empty, '/vive/set_origin', callback_group=callback_group)

        self.tracker_pose = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.target_frame = 'tracker_LHR_A5314E9D'
        #self.timer = self.create_timer(0.05, self.tracker_pose_read, callback_group=callback_group)

        self.path_to_traj = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'data.csv')
        self.goal_trajectory = []

        self.path_to_data = os.path.expandvars('$HOME') + '/calibration/experiments'
        self.path_to_config = os.path.expandvars('$HOME') + '/calibration/config/ar_20.json'

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.base_calibration_srv = self.create_service(Empty, "/base_frame_calibration", self.base_calibration_cb, callback_group=callback_group)
        self.start_experiments_srv = self.create_service(Empty, "/start_experiment", self.start_experiment_cb, callback_group=callback_group)


        self.loop_rate = self.create_rate(0.5, self.get_clock())

    def make_transform(self, translation, quat):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_frame'

        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

    
        self.tf_static_broadcaster.sendTransform(t)

    def get_tracker_pose(self):
        from_frame = 'world'
        to_frame = self.target_frame
        t = self.tf_buffer.lookup_transform()

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
        
        Rt = np.append(rotation.as_matrix(), translation, axis=1)
        tracker_pose = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])
        return tracker_pose


    def get_trajectory(self):

        trajectory_array = []
        with open(self.path_to_traj, 'r') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                lst = map(float, row)
                trajectory_array.append(list(lst)[:6])  
        
        return trajectory_array


    # def tracker_pose_read(self):
    #     from_frame = 'world'
    #     to_frame = self.target_frame

    #     try:
    #         t = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
    #     except TransformException as ex:
    #         self.get_logger().info(f'Could not transform {to_frame} to {from_frame}: {ex}')
    #         return
    
    #     rotation = R.from_quat(np.array([t.transform.rotation.x,
    #                                      t.transform.rotation.y,
    #                                      t.transform.rotation.z,
    #                                      t.transform.rotation.w]))    
    #     translation = np.array([[t.transform.translation.x],
    #                             [t.transform.translation.y],
    #                             [t.transform.translation.z]])
        
    #     Rt = np.append(rotation.as_matrix(), translation, axis=1)
    #     self.tracker_pose = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])
        # self.pub.publish(translation.tolist())
        
    def get_tracker_pose_array(self):
        rotation_mat = R.from_matrix(self.tracker_pose[:3, :3])
        translation_vec = self.tracker_pose[0:3, [3]]
        result = np.concatenate([translation_vec, rotation_mat.as_euler('zyx', degrees=False)])
        return result.tolist()



    def base_calibration(self):
        initial_pos = [0.0, 0.0, math.radians(90.0), 0.0, math.radians(90.0), 0.0]

        req = MoveJ.Request()
        req.angles = initial_pos
        future = self.movej_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # time.sleep(2.0)
        self.loop_rate.sleep()


        future = self.set_origin_cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)

        tool_params = self.tool_subscription.read()[0].value
        tool_rotation = R.from_euler('zyx', np.array(tool_params[3:]))
        tool_trans = tool_params[:3]
        Rt = np.append(tool_rotation.as_matrix(), [[tool_trans[0]], [tool_trans[1]], [tool_trans[2]]], axis=1)
        tool_tf = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])

        tracker_tf = self.get_tracker_pose()
        base_frame_tf = tracker_tf @ inv(tool_tf)
        
        ### write base frame tf to json

        base_frame_rot = R.from_matrix(base_frame_tf[:3, :3])
        base_frame_trans = base_frame_tf[0:3, [3]].flatten()
        base_pose = np.concatenate([base_frame_trans, base_frame_rot.as_euler('zyx')])


        ### publish static transform from nominal base frame to vive frame
        self.make_transform(base_frame_trans, base_frame_rot.as_quat())


        ### write calibrate base frame to config json file
        with open(self.path_to_config, "r", encoding='utf-8') as f:
            data = json.load(f)
        data['nominal_base_params'] = base_pose.tolist()

        with open(self.path_to_config, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)


        self.get_logger().info("Calibration of the base frame has been done")

    


    def execute_experiment(self):
        self.get_logger().info("Starting experiment...")
        
        goal_trajectory = self.get_trajectory()

        for traj in goal_trajectory:
            req = MoveJ.Request()
            req.angles = traj
            future = self.movej_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            self.loop_rate.sleep()

            folder = Path(self.path_to_data)
            file_count = len(list(folder.iterdir()))

            header = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6',
              'px', 'py', 'pz', 'Rz', 'Ry', 'Rx']
            
            with open(self.path_to_data + "\experiment" + str(file_count) + ".csv", "w", encoding='UTF8') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                actual_joint_position = self.joint_subscription.read()[0].value
                tracker_pose = self.get_tracker_pose()

                tracker_trans = tracker_pose[0:3, [3]].flatten()
                tracker_rot = R.from_matrix(tracker_pose[:3, :3])
                tracker_position = np.concatenate([tracker_trans, tracker_rot.as_euler('zyx')])

                data = actual_joint_position + tracker_position.tolist()
                writer.writerow(data)

                self.get_logger().info(f"Write point with coordinates: x={tracker_position[0]}, y={tracker_position[1]}, z={tracker_position[2]}, Rz={tracker_position[3]}, Ry={tracker_position[4]}, Rx{tracker_position[5]}")
    
    
    def base_calibration_cb(self, request, response):
        print("call srv")
        self.base_calibration()
        print("done")
        return Empty.Response()

    def start_experiment_cb(self, request, response):
        self.execute_experiment()
        return



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