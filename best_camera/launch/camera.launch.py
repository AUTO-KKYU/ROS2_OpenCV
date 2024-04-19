import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera_node = Node(
        package='best_camera',
        executable='camera_node',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'frame_width': 640},
            {'frame_height': 480},
            {'frequency': 15.0},
            {'frame_id': 'camera'},
            {'reliability': 'reliable'},
            {'history': 'keep_last'},
            {'depth': 5}
        ]
    )

    picture_node = Node(
        package='best_camera',
        executable='picture_node',
        name='picture_node',
        output='screen',
        emulate_tty=True
    )

    ld.add_action(camera_node)
    ld.add_action(picture_node)

    return ld
