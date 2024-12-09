from launch import LaunchDescription
from launch_ros.actions import Node
from math import pi, degrees, radians


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pcd_publisher',
            executable='pcd_subscriber',
            output='screen',
            parameters=[
                {"pcd_file_path": "/home/he/s99.pcd"},
                {"frame_id": "map_gyor_0"},
                {"topic": "/pointcloud"},
            ],
        ),        
    ])