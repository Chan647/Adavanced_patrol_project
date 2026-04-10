# launch/detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pt_pkg',
            executable='fire',
            name='fire_detection_node',
            output='screen',
        ),
        Node(
            package='pt_pkg',
            executable='person',
            name='person_detection_node',
            output='screen',
        ),
    ])