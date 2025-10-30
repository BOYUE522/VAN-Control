import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def launch_setup(context, *args, **kwargs):
    with open(LaunchConfiguration('param_file').perform(context), 'r') as f:
        all_params = yaml.safe_load(f)
        driver_params = all_params['oxts_driver']['ros__parameters']
        ins_params = all_params['oxts_ins']['ros__parameters']

    oxts_driver_node = Node(
        package='oxts_driver',
        executable='oxts_driver',
        name='oxts_driver',
        output='screen',
        parameters=[
            driver_params,
            {'unit_ip': LaunchConfiguration('unit_ip')},
            {'wait_for_init': True},
            {'topic_prefix': LaunchConfiguration('topic_prefix')},
        ]
    )

    oxts_ins_node = Node(
        package='oxts_ins',
        executable='oxts_ins',
        name='oxts_ins',
        output='screen',
        parameters=[
            ins_params,
            {'topic_prefix': LaunchConfiguration('topic_prefix')},
        ],
    )

    return [oxts_driver_node, oxts_ins_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('unit_ip', default_value='0.0.0.0', description='IP address for the OxTS receiver'),
        DeclareLaunchArgument('topic_prefix', default_value='', description='Prefix for topics'),
        DeclareLaunchArgument('param_file', default_value=os.path.join(get_package_share_directory('uw'), 'config', 'oxts_params.yaml'), description='Full path to configuration parameter file'),
        OpaqueFunction(function=launch_setup)
    ])
