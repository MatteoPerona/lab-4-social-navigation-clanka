from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    map_path = LaunchConfiguration('map')
    start = LaunchConfiguration('start')
    mid = LaunchConfiguration('mid')
    final = LaunchConfiguration('final')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')

    tb4_nav_share = get_package_share_directory('turtlebot4_navigation')
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_share, 'launch', 'nav_bringup.launch.py')),
        launch_arguments={
            'slam': 'off',
            'localization': 'true',
            'map': map_path
        }.items()
    )

    waypoints_node = Node(
        package='lab4',
        executable='social_navigation_node',
        name='social_navigation_node',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'publish_initial_pose': publish_initial_pose,
            'start': [0.0, 0.0, 0.0],  # will be overridden by CLI if provided
            'mid':   [1.0, 0.0, 0.0],
            'final': [2.0, 0.0, 0.0],
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('map',  default_value='/home/ubuntu/donut-killer.yaml'),
        DeclareLaunchArgument('publish_initial_pose', default_value='true'),
        DeclareLaunchArgument('start', default_value='[0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('mid',   default_value='[1.0, 0.0, 0.0]'),
        DeclareLaunchArgument('final', default_value='[2.0, 0.0, 0.0]'),
        nav_bringup,
        waypoints_node,
    ])
