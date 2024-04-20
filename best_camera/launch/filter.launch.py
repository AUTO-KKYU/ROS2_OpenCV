import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 
    prarm_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
        get_package_share_directory('best_camera'),
        'param',
        'filter.yaml')
        )
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
            'param_dir',
            default_value=prarm_dir
            ),

            Node(
                package='best_camera',
                executable='edge',
                name='edge',
                parameters=[prarm_dir],
                output='screen'),

            Node(
                package='best_camera',
                executable='img_control',
                name='img_control',
                parameters=[prarm_dir],
                output='screen'),

            Node(
                package='best_camera',
                executable='optical_flow',
                name='optical_flow',
                parameters=[prarm_dir],
                output='screen'),

            Node(
                package='best_camera',
                executable='cartoon',
                name='cartoon',
                parameters=[prarm_dir],
                output='screen'),

            Node(
                package='best_camera',
                executable='gray',
                name='gray',
                parameters=[prarm_dir],
                output='screen'),

            Node(
                package='best_camera',
                executable='aruco',
                name='aruco',
                parameters=[prarm_dir],
                output='screen'),
    ])


