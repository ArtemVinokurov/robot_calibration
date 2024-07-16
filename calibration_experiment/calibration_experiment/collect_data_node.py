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



class GetTrackerPoseSrv(Node):
    def __init__(self):
        super().__init__("get_pose_srv")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.target_frame = 'tracker_LHR_A5314E9D'


        cb_group = ReentrantCallbackGroup()
        self.create_service(TrackerPose, "/get_tracker_pose", self.get_pose_cb,callback_group=cb_group)

    def get_pose_cb(self, request: TrackerPose.Request, response: TrackerPose.Response):
        from_frame = 'world_vive'

        try:
            t = self.tf_buffer.lookup_transform(from_frame, self.target_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.target_frame} to {from_frame}: {ex}')
            response.success = False
            return
        response.success = True
        response.tf = t.transform
        return response



class DataCollectionNode(Node):
    def __init__(self):
        super().__init__("data_collection_node")

        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'mcx.cert.pem')

        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.2.100:5568:5567', self.motorcortex_types, parameter_tree,
                                                     certificate=license_file, timeout_ms=10000, login="admin", password="vectioneer")
        
        # self.subscription = self.sub
            self.joint_subscription = self.sub.subscribe(
                ['root/ManipulatorControl/jointPositionsActual'], 'group1', 1)
            self.tool_subscription = self.sub.subscribe('root/ManipulatorControl/manipulatorToolPoseActual', 'group2', 1)
            self.tool_subscription.get()
            self.joint_subscription.get()            
        except Exception as e:
            self.get_logger().error(f"Failed to establish connection: {e}")
            return
        
        cb_group = ReentrantCallbackGroup()

        self.movej_cli = self.create_client(MoveJ, 'moveJ', callback_group=cb_group)
        # self.set_origin_cli = self.create_client(Empty, '/vive/set_origin', callback_group=cb_group)
        self.get_tracker_pose_cli = self.create_client(TrackerPose, '/get_tracker_pose', callback_group=cb_group)

        while self.movej_cli.wait_for_service() and self.get_tracker_pose_cli.wait_for_service():
            self.get_logger().info("Services not available, wait...")

        self.generally_data = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'data.csv')

        self.base_calibrate_data = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'base_data.csv')
        self.tool_calibrate_data = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'tool_data.csv')

        self.goal_trajectory = []

        self.path_to_data = os.path.expandvars('$HOME') + '/calibration/experiments'
        self.path_to_config = os.path.expandvars('$HOME') + '/calibration_new/ar_20.json'

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.base_calibration_srv = self.create_service(Empty, "/base_frame_calibration", self.base_calibration_cb, callback_group=cb_group)
        self.start_experiments_srv = self.create_service(Empty, "/start_experiment", self.start_experiment_cb, callback_group=cb_group)
        self.base_calibration_srv = self.create_service(Empty, "/start_base_experiment", self.base_experiment_cb, callback_group=cb_group)
        self.start_experiments_srv = self.create_service(Empty, "/start_tool_experiment", self.tool_experiment_cb, callback_group=cb_group)

    def make_transform(self, translation, quat):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world_vive'
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
        from_frame = 'world_vive'
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


    def get_trajectory(self, data, type_experiment):
        trajectory_array = []
        with open(data, 'r', newline='') as f:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                lst = list(map(float, row))
                if len(lst) == 0:
                    continue
                trajectory_array.append(lst.copy()[:6])
                if type_experiment == '/base' or type_experiment == '/tool':
                    trajectory_array[-1].append(lst[-1])  
        
        return trajectory_array

        
    # def get_tracker_pose_array(self):
    #     rotation_mat = R.from_matrix(self.tracker_pose[:3, :3])
    #     translation_vec = self.tracker_pose[0:3, [3]]
    #     result = np.concatenate([translation_vec, rotation_mat.as_euler('zyx', degrees=False)])
    #     return result.tolist()


    async def base_calibration(self):
        initial_pos = [0.0, 0.0, math.radians(90.0), 0.0, math.radians(90.0), 0.0]

        req = MoveJ.Request()
        req.angles = initial_pos
        #await self.movej_cli.call_async(req)
        # time.sleep(2.0)
        #self.loop_rate.sleep()
        #await self.set_origin_cli.call_async(Empty.Request())

        tool_params = self.tool_subscription.read()[0].value
        tool_rotation = R.from_euler('ZYX', np.array(tool_params[3:]))
        tool_trans = tool_params[:3]
        Rt = np.append(tool_rotation.as_matrix(), [[tool_trans[0]], [tool_trans[1]], [tool_trans[2]]], axis=1)
        tool_tf = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])

        future = self.get_tracker_pose_cli.call_async(TrackerPose.Request())
        tracker_pose = await future

        if not tracker_pose.success:
            self.get_logger().error("Failed to get tracker pose")
            return

        tracker_tf = self.pose_to_homogeneous(tracker_pose.tf)

        # base_frame_tf = tracker_tf @ np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0.01881], [0, 0, 0, 1]]) @ inv(tool_tf)
        z_rot = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        y_rot = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # base_frame_tf = tracker_tf @ inv(tool_tf)
        base_frame_tf = tracker_tf

        

        ### write base frame tf to json

        base_frame_rot = R.from_matrix(base_frame_tf[:3, :3])
        base_frame_trans = base_frame_tf[0:3, 3].flatten()
        base_pose = np.concatenate([base_frame_trans, base_frame_rot.as_euler('zyx')])


        ### publish static transform from nominal base frame to vive frame
        self.make_transform(base_frame_trans, base_frame_rot.as_quat())

        base_manip_translation = np.asarray(tool_trans) - base_frame_trans
        print(base_manip_translation)
        ### write calibrate base frame to config json file
        with open(self.path_to_config, "r", encoding='utf-8') as f:
            data = json.load(f)
        data['nominal_base_params'] = base_pose.tolist()
        data['zero_tracker_position'] = base_manip_translation.tolist()

        with open(self.path_to_config, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)


        self.get_logger().info("Calibration of the base frame has been done")


    async def execute_experiment(self, type_experiment, data):
        self.get_logger().info("Starting experiment...")

        folder = Path(self.path_to_data+type_experiment)
        file_count = len(list(folder.iterdir()))

        if type_experiment == "/base" or type_experiment == "/tool":
            header = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6',
            'px_r', 'py_r', 'pz_r', 'joint']
        else: 
            header = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6',
                'px_r', 'py_r', 'pz_r', 'rx_r', 'ry_r', 'rz_r']

        with open(self.path_to_data + type_experiment + "/experiment" + str(file_count+1) + ".csv", "w", encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(header)        

            goal_trajectory = self.get_trajectory(data, type_experiment)

            number_of_point = 0
            for traj in goal_trajectory:
                req = MoveJ.Request()
                req.angles = traj[:6]
                await self.movej_cli.call_async(req)
                time.sleep(2.0)

                actual_joint_position = self.joint_subscription.read()[0].value
                future = self.get_tracker_pose_cli.call_async(TrackerPose.Request())
                tracker_pose = await future

                if not tracker_pose.success:
                    continue

                tracker_tf = self.pose_to_homogeneous(tracker_pose.tf)
                tracker_trans = tracker_tf[0:3, [3]].flatten()
                tracker_rot = R.from_matrix(tracker_tf[:3, :3])
                tracker_position = np.concatenate([tracker_trans, tracker_rot.as_euler('zyx')])

                if type_experiment == '/base' or type_experiment == '/tool':
                    data = list(actual_joint_position) + tracker_trans.tolist()[:3] + traj[6]
                else:
                    data = list(actual_joint_position) + tracker_position.tolist()
                writer.writerow(data)
                number_of_point += 1

                self.get_logger().info(f"Write point №{number_of_point} with coordinates: x={tracker_position[0]}, y={tracker_position[1]}, z={tracker_position[2]}, Rz={tracker_position[3]}, Ry={tracker_position[4]}, Rx{tracker_position[5]}")
        
        self.get_logger().info("End experiment")
    
    async def base_calibration_cb(self, request, response):
        await self.base_calibration()
        return response

    async def start_experiment_cb(self, request, response):
        await self.execute_experiment("/generally", self.generally_data)
        return response
    
    async def base_experiment_cb(self, req, res):
        await self.execute_experiment("/base", self.base_calibrate_data)
        return res
    
    async def tool_experiment_cb(self, req, res):
        await self.execute_experiment("/tool", self.tool_calibrate_data)
        return res



    def pose_to_homogeneous(self, pose : Transform):
        rotation = R.from_quat(np.array([pose.rotation.x,
                                         pose.rotation.y,
                                         pose.rotation.z,
                                         pose.rotation.w]))
            
        translation = np.array([[pose.translation.x],
                                [pose.translation.y],
                                [pose.translation.z]])
        Rt = np.append(rotation.as_matrix(), translation, axis=1)
        tracker_tf = np.vstack([Rt, np.array([0.0, 0.0, 0.0, 1.0])])
        return tracker_tf



def main():
    rclpy.init()
    main_node = DataCollectionNode()
    get_pose_node = GetTrackerPoseSrv()

    executor = SingleThreadedExecutor()
    executor.add_node(main_node)
    executor.add_node(get_pose_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, Exception):
        main_node.get_logger().info('Shutting down..')

    
    main_node.destroy_node()
    get_pose_node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()