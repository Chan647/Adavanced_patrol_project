import launch
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pt_pkg_dir   = get_package_share_directory('pt_pkg')
    pt_pypkg_dir = get_package_share_directory('pt_pypkg')

    patrol_node = Node(
        package='pt_pkg',
        executable='patrol_nav_node',
        name='patrol_nav_node',
        output='screen',
    )

    detection_node = Node(
        package='pt_pkg',
        executable='test_detection',
        name='detection_node',
        output='screen',
    )

    voice_control_node = Node(
        package='ljw_cpp_pkg',
        executable='voice_control',
        name='voice_control_node',
        output='screen',
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
    )

    map_slam_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pt_pkg_dir, 'launch', 'map.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        patrol_node,
        detection_node,
        voice_control_node,
        web_video_server_node,
        map_slam_launch,
    ])