from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from math import pi, degrees, radians


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pcd_publisher',
            executable='pcd_publisher',
            output='screen',
            parameters=[
                {"pcd_file_path": "/home/he/dlio_map2.pcd"},
                {"frame_id": "map_gyor_0"},
                {"rate": 1},
                {"x_translation": -24.0},
                {"y_translation": 77.0},
                {"z_translation": 0.0},
                {"x_rotation": 0.0},
                {"y_rotation": 0.0},
                {"z_rotation": radians(-177.1)}, # -177.1 deg is -3.09097 rad
            ],
        ),
        # Node(
        #     package='rqt_reconfigure',
        #     executable='rqt_reconfigure',
        # ),
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
        # Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        #     parameters=[
        #         {'port': 8765},
        #         {'address': '0.0.0.0'},
        #         {'tls': False},
        #         {'certfile': ''},
        #         {'keyfile': ''},
        #         #{'topic_whitelist': "'.*'"},
        #         {'max_qos_depth': 10},
        #         {'num_threads': 0},
        #         {'use_sim_time': False},
        #         {'send_buffer_limit': 100000000 }, # 100MB because of the large point cloud
        #     ]
        # ),
        # Node(
        #     package='lanelet2_rviz2',
        #     executable='visualize_osm',
        #     output='screen',
        #     parameters=[
        #         {"frame_id": "map_gyor_0"},
        #         {"line_width": 0.4},
        #         {"osm_filename": "/home/he/gyor_uni.osm"},
        #         {"center_map": False},
        #         {"speed_color_max": 90.0}
        #     ],
        # ),        
    ])