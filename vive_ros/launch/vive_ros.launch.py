import os
import yaml
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

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
    # config_yaml = load_yaml('vive_ros', 'config', 'parameters.yaml')
    config_yaml = os.path.join(get_package_share_directory('vive_ros'), 'config', 'params.yaml')
    print(config_yaml)
    return LaunchDescription([
        Node(
            package='vive_ros',
            executable='vive_node',
            output='screen',
            parameters=[config_yaml]
        )
    ])