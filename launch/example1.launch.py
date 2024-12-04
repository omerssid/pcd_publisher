from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pcd_publisher',
            executable='pcd_publisher',
            output='screen',
            parameters=[
                {"pcd_file_path": "/home/he/dlio_map2.pcd"},
                {"frame_id": "map"},
                {"rate": 1},
                {"x_translation": 0.0},
                {"y_translation": 0.0},
                {"z_translation": 0.0},
                {"x_rotation": 0.0},
                {"y_rotation": 0.0},
                {"z_rotation": 0.0},
            ],
        ),
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gyor0_tf_publisher',
            output='screen',
            arguments=[
                '--x',  '697237.0',
                '--y',  '5285644.0',
                '--z',  '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',      'map',
                '--child-frame-id','map_gyor_0'
            ],
        ),
    ])