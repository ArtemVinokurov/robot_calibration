#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import signal
import math
from geometry_msgs.msg import Twist

import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped, Transform
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_share_directory
import os

import io, json

def raise_timeout(signum, frame):
    raise TimeoutError


class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.read_tf)

        self.target_frame = 'tracker_LHR_A5314E9D'
        self.pub = self.create_publisher(Transform, 'tracker_tf', 5)

        self.tf_list = []
        self.path_to_json = os.path.expandvars('$HOME') + '/temp/data.json'

    def read_tf(self):
        from_frame = 'world'
        to_frame = self.target_frame

        try:
            t = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame} to {from_frame}: {ex}')
            return
        
        tf = t.transform
        self.tf_list.append(tf)
        self.pub.publish(tf)

    

    def write_data(self):
        js_path = self.path_to_json
        with open(js_path, "r", encoding='utf-8') as f:
            present_data = f.read()

        dict_data = json.loads(present_data)

        traj_index = len(dict_data["trajectories"]) + 1
        trajectory_dict = {"name": "trajectory_" + str(traj_index), "type": "work", "transform": []}
        for tf in self.tf_list:
            x = tf.translation.x
            y = tf.translation.y
            z = tf.translation.z
            qw = tf.rotation.w
            qx = tf.rotation.x
            qy = tf.rotation.y
            qz = tf.rotation.z

            trajectory_dict["transform"].append({"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw})
        
        dict_data["trajectories"].append(trajectory_dict)

        with open(js_path, "w", encoding='utf-8') as f:
            json.dump(dict_data, f)


def main():

    signal.signal(signal.SIGALRM, raise_timeout)

    signal.alarm(20)

    rclpy.init()
    read_node = FrameListener()

    try:
        rclpy.spin(read_node)
    finally:
        signal.alarm(0)
        read_node.write_data()
        read_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        


        




    