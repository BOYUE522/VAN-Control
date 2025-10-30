import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    ntrip_params = LaunchConfiguration('param_file')
    ntrip_forwarding_node = Node(
        package='oxts_ntrip',
        executable='ntrip_forwarding.py',
        name='ntrip_forwarding',
        output='screen',
        parameters=[ntrip_params]
    )

    gga_generator_node = Node(
        package='oxts_ntrip',
        executable='oxts_ntrip_gga',
        name='gga_generator',
        remappings=[
            ('fix', LaunchConfiguration('fix_topic'))
        ]
    )

    return [ntrip_forwarding_node, gga_generator_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fix_topic', default_value='/oxts/fix', description='Fix topic used to generate and send GGA messages to NTRIP server'),
        DeclareLaunchArgument('param_file',
                              default_value=os.path.join(get_package_share_directory('oxts_ntrip'), 'config', 'ntrip_params.yaml'),
                              description='Full path to configuration parameter file with NTRIP credentials'),
        OpaqueFunction(function=launch_setup)
    ])