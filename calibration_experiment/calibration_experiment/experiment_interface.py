#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
import sys

class ClientNode(Node):
    def __init__(self):
        super().__init__('experiment_interface')
        self.base_calibration_cli = self.create_client(Empty, "/base_frame_calibration")
        self.start_experiment_cli = self.create_client(Empty, "/start_experiment")
        self.base_experiment_cli = self.create_client(Empty, "/start_base_experiment")
        self.tool_experiment_cli = self.create_client(Empty, "/start_tool_experiment")

        while not self.base_calibration_cli.wait_for_service(timeout_sec=1.0) \
              or not self.start_experiment_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Collect data node not ready, wait..")
        
    
    def start_experiments_req(self):
        req = Empty.Request()
        return self.start_experiment_cli.call_async(req)
    
    def base_calibration_req(self):
        req = Empty.Request()
        return self.base_calibration_cli.call_async(req)
    
    def start_base_req(self):
        req = Empty.Request()
        return self.base_experiment_cli.call_async(req)
    
    def start_tool_req(self):
        req = Empty.Request()
        return self.tool_experiment_cli.call_async(req)


def main():
        
    rclpy.init()
    node = ClientNode()

    
    while rclpy.ok():
        print("1. Calibration base frame \n2. Base experiment\n3. Tool experiment\n4. Generally experiment")
        command = int(input())
        if command == 1:
            future = node.base_calibration_req()
            rclpy.spin_until_future_complete(node, future)
        elif command == 2:
            future = node.start_base_req()
            rclpy.spin_until_future_complete(node, future)
        elif command == 3:
            future = node.start_tool_req()
            rclpy.spin_until_future_complete(node, future)
        elif command == 4:
            future = node.start_experiments_req()
            rclpy.spin_until_future_complete(node, future)


    rclpy.shutdown()


if __name__ == '__main__':
    main()


    

