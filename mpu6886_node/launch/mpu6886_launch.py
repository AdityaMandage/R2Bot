from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpu6886_node',
            executable='mpu6886_node',
            name='mpu6886_node'
        )
    ])
