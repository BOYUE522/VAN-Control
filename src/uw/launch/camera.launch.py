import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import subprocess

def run_camera_config(context, *args, **kwargs):
    config_file_str = LaunchConfiguration('param_file').perform(context)

    subprocess.run(["ros2", "run", "ds_aravis_cli", "ds_aravis_cli", config_file_str])

    with open(config_file_str, 'r') as f:
        all_params = yaml.safe_load(f)
        aravis_camera_id = all_params['name']
        image_width = all_params['width']
        image_height = all_params['height']
        frame_rate = all_params['frame_rate']

        do_compression_str = LaunchConfiguration('jpeg_compression').perform(context)
        upside_down_str = LaunchConfiguration('upside_down').perform(context)
        do_compression = do_compression_str == 'true' or do_compression_str == 'True'
        upside_down = upside_down_str == 'true' or upside_down_str == 'True'
        if do_compression:
            if upside_down:
                pipeline_str = f'aravissrc camera-name="{aravis_camera_id}" do-timestamp=true ! video/x-bayer,format=rggb,width={image_width},height={image_height},framerate={frame_rate}/1 ! bayer2rgb ! videoflip method=rotate-180 ! cameraundistort ! videoconvert ! video/x-raw,format=RGB ! queue max-size-buffers=3 leaky=downstream ! jpegenc ! appsink'
            else:
                pipeline_str = f'aravissrc camera-name="{aravis_camera_id}" do-timestamp=true ! video/x-bayer,format=rggb,width={image_width},height={image_height},framerate={frame_rate}/1 ! bayer2rgb ! cameraundistort ! videoconvert ! video/x-raw,format=RGB ! queue max-size-buffers=3 leaky=downstream ! jpegenc ! appsink'
        else:
            if upside_down:
                pipeline_str = f'aravissrc camera-name="{aravis_camera_id}" do-timestamp=true ! video/x-bayer,format=rggb,width={image_width},height={image_height},framerate={frame_rate}/1 ! bayer2rgb ! videoflip method=rotate-180 ! cameraundistort ! videoconvert ! video/x-raw,format=RGB ! queue max-size-buffers=3 leaky=downstream ! appsink'
            else:
                pipeline_str = f'aravissrc camera-name="{aravis_camera_id}" do-timestamp=true ! video/x-bayer,format=rggb,width={image_width},height={image_height},framerate={frame_rate}/1 ! bayer2rgb ! cameraundistort ! videoconvert ! video/x-raw,format=RGB ! queue max-size-buffers=3 leaky=downstream ! appsink'
        camera_info_file_str = f'camera_intrinsics/{LaunchConfiguration("camera_name").perform(context)}_info.yaml'

        ds_video_node = Node(
            package='ds_video_gst',
            executable='ds_video_gst_node',
            name='gst_rgb',
            namespace=LaunchConfiguration('camera_name'),
            output='screen',
            parameters=[
                {'pipeline': pipeline_str},
                {'camera_info_pkg': 'uw'},
                {'camera_info_yaml': camera_info_file_str},
                {'frame_id': LaunchConfiguration('camera_name')},
                {'image_topic': 'image_rect'}
            ]
        )

        return [ds_video_node]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value=os.path.join(get_package_share_directory('uw'), 'camera_settings', 'camera.json'), description='Full path to configuration parameter file'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('jpeg_compression', default_value='true'),
        DeclareLaunchArgument('upside_down', default_value='false'),
        OpaqueFunction(function=run_camera_config),
    ])
