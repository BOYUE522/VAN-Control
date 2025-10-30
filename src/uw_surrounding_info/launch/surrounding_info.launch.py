from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uw_surrounding_info',
            executable='surrounding_info_node',
            name='surrounding_info_node',
            output='screen',
            parameters=[
                {
                    'base_frame': 'base_footprint',
                    'scan_topics': [
                        '/front_lidar/scan',
                        '/rl_lidar/scan',
                        '/rr_lidar/scan',
                    ],
                    'max_distance': 30.0,
                    'num_bins': 240,
                    'publish_frequency': 10.0,
                    'path_length': 100,
                    'path_step': 0.3,
                }
            ],
        ),
    ])
