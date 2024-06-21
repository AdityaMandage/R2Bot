import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='articubot_one',
            executable='launch_robot',
            name='launch_robot',
            output='screen'
        ),
        Node(
            package='articubot_one',
            executable='camera',
            name='camera',
            output='screen'
        ),
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),
        Node(
            package='autonomy_package',
            executable='ball_mover_node',
            name='ball_mover_node',
            output='screen'
        )
    ])
