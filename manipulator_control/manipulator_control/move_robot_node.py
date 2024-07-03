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
# from robot_control.motion_program import Waypoint, MotionProgram, PoseTransformer
# from robot_control.robot_command import RobotCommand
# from robot_control.system_defs import InterpreterStates
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os



class MoveRobotService(Node):
    def __init__(self):
        super().__init__('move_robot_service')
        self.movej_srv_ = self.create_service(MoveJ, 'moveJ', self.move_robot_callback)
        parameter_tree = motorcortex.ParameterTree()
        self.motorcortex_types = motorcortex.MessageTypes()
        license_file = os.path.join(get_package_share_directory('manipulator_control'), 'resource', 'mcx.cert.pem')

        try:
            self.req, self.sub = motorcortex.connect('wss://192.168.57.3:5568:5567', self.motorcortex_types, parameter_tree,
                                                     certificate=license_file, timeout_ms=1000, login="admin", password="vectioneer")
        
        # self.subscription = self.sub
            self.joint_subscription = self.sub.subscribe(
                ['root/ManipulatorControl/jointPositionsActual'], 'group1', 5)
            self.joint_subscription.get()            
        except Exception as e:
            self.get_logger().error(f"Failed to establish connection: {e}")
            return
        
        self.robot = RobotCommand(self.req, self.motorcortex_types)

        if self.robot.engage():
            self.get_logger().info('Robot is at Engage')
        else:
            self.get_logger().error('Failed to set robot to Engage')
            return
        
        self.robot.reset()

    def move_robot_callback(self, request: MoveJ.Request, response: MoveJ.Response):
        angles = request.angles
        # joint_params = self.joint_subscription.read()
        # ref_joint_value = joint_params[0].value
        # start_pos = Waypoint(ref_joint_value)
        # jpos = Waypoint(angles)
        # motion_prog = MotionProgram(self.req, self.motorcortex_types)
        # motion_prog.addMoveJ([start_pos, jpos], 1.0, 1.0)
        # motion_prog.send("ptp").get()
        # if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
        #     if self.robot.moveToStart(5):
        #         print('Robot is at the start position')
        #     else:
        #         raise Exception('Failed to move to the start position')
        #     self.robot.play()

        # while self.robot.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
        #     time.sleep(0.1)
        
        # while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
        #     pass

        result = self.robot.moveToPoint(list(angles), 1.0, 1.0)
        response.success = result
        return response
    
def main(args=None):
    rclpy.init(args=args)
    move_point_node = MoveRobotService()
    
    rclpy.spin(move_point_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        


