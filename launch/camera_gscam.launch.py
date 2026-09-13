from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    port = LaunchConfiguration('port').perform(context)
    decoder = LaunchConfiguration('decoder').perform(context)
    latency = LaunchConfiguration('rtp_latency_ms').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)

    gscam_config = (
        f'udpsrc port={port} ! '
        f'application/x-rtp,media=video,encoding-name=H264,payload=96 ! '
        f'rtpjitterbuffer latency={latency} ! '
        f'rtph264depay ! h264parse ! {decoder} ! videoconvert'
    )
# ros2 launch bluerov2_controller camera_gscam.launch.py decoder:=vaapih264dec  
    camera_node = Node(
        package='gscam2',
        executable='gscam_main',
        name='camera_streamer',
        namespace=namespace,
        output='screen',
        parameters=[{
            'gscam_config': gscam_config,
            'sync_sink': False,
            'preroll': False,
            'use_gst_timestamps': True,
            'camera_name': 'bluerov2',
            'frame_id': frame_id,
        }],
    )

    return [camera_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument(
            'port', default_value='5600',
            description='UDP port for H.264 RTP stream from BlueOS'),
        DeclareLaunchArgument(
            'decoder', default_value='avdec_h264',
            description='H.264 decoder: avdec_h264 (CPU), vaapih264dec (Intel), nvh264dec (NVIDIA)'),
        DeclareLaunchArgument(
            'rtp_latency_ms', default_value='50',
            description='rtpjitterbuffer latency in ms'),
        DeclareLaunchArgument(
            'frame_id', default_value='bluerov2_camera'),
        OpaqueFunction(function=launch_setup),
    ])
