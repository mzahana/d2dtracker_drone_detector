from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='d2dtracker_drone_detector',
            namespace='detection_node',
            executable='detect',
        )
    ])