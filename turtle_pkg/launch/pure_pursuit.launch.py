from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file = PathJoinSubstitution([
        FindPackageShare('turtle_pkg'),
        'config',
        'pure_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='turtle_pkg',
            executable='pure_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[param_file]
        )
    ])
