#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown, RegisterEventHandler)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.conditions import IfCondition, LaunchConfigurationEquals

def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)
    print(absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print("error yaml file")
        return None
    



def generate_launch_description():


    ### HTC Vive node
    vive_config = os.path.join(get_package_share_directory('vive_ros'), 'config', 'params.yaml')
    vive_node = Node(
        package='vive_ros',
        executable='vive_node',
        output='screen',
        parameters=[vive_config]
    )

    collect_data_node = Node(
        package='calibration_experiment',
        executable='collect_data_node.py',
        output='screen'
    )

    manipulator_control = Node(
        package='manipulator_control',
        executable='move_robot_node.py',
        output='screen'
    )

    experiment_interface = Node(
        package="calibration_experiment",
        executable="experiment_interface.py",
        output='screen'
    )


    return LaunchDescription(
        [
            vive_node,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=vive_node,
                    on_start=[manipulator_control, collect_data_node]
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=collect_data_node,
                    on_start=[experiment_interface]
                )
            )
        ]
    )