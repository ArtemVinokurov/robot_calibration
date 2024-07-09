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
from ament_index_python.packages import get_package_share_directory
import os
import csv

class MoveClinet(Node):
    def __init__(self):
        super().__init__("move_client")
        self.cli = self.create_client(MoveJ, 'moveJ')

        self.path_to_data = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'data.csv')
        self.goal_trajectory = []
    
    def get_trajectory(self):
        with open(self.path_to_data, 'r') as f:
            reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
            print(reader)
            for row in reader[1:]:
                self.goal_trajectory.append(row[:6])
    
    def execute_trajecoty(self):
        self.get_trajectory()
        for traj in self.goal_trajectory:
            req = MoveJ.Request()
            req.angles = traj
            self.cli.call(req)
            rclpy.spin_once()
            time.sleep(0.5)



def main():
    rclpy.init()
    client_node = MoveClinet()

    client_node.execute_trajecoty()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
