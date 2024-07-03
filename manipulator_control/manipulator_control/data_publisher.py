#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import pi, asin, cos, sin, acos, atan2, sqrt, radians
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation as R
from robot_interfaces.srv import MoveJ


import motorcortex
import math
import time
from .robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
from .robot_control.robot_command import RobotCommand
from .robot_control.system_defs import InterpreterStates
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float64MultiArray


class ReadDataNode(Node):
    def __init__(self):
        super().__init__('read_robot_data')

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
        
        self.tool_pub = self.create_publisher(Float64MultiArray, "toolPos", qos_profile=5)
        self.joint_pub = self.create_publisher(Float64MultiArray, "jointValue", qos_profile=5)

        self.timer = self.create_timer(0.5, self.read_data_callback)

    def read_data_callback(self):
        tool_params = self.tool_subscription.read()
        tool_pos_value = tool_params[0].value

        joint_params = self.joint_subscription.read()
        joint_pos_value = joint_params[0].value

        tool_pos_msg = Float64MultiArray()
        joint_pos_msg = Float64MultiArray()

        tool_pos_msg.data = tool_pos_value
        joint_pos_msg.data = joint_pos_value

        self.tool_pub.publish(tool_pos_msg)
        self.joint_pub.publish(joint_pos_msg)


def main():
    rclpy.init()

    read_node = ReadDataNode()

    rclpy.spin(read_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
