from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'node_name',
            default_value='face_detection',
            description='Name of the node'
        ),
        DeclareLaunchArgument(
            'image',
            default_value='image',
            description='The image topic. Should be remapped to the name of the real image topic.'
        ),
        DeclareLaunchArgument(
            'use_camera_info',
            default_value='false',
            description='Indicates that the camera_info topic should be subscribed to get the default input_frame_id'
        ),
        DeclareLaunchArgument(
            'debug_view',
            default_value='true',
            description='Specify whether the node displays a window to show edge image'
        ),
        DeclareLaunchArgument(
            'queue_size',
            default_value='3',
            description='Specify queue_size of input image subscribers'
        ),
        DeclareLaunchArgument(
            'face_cascade_name',
            default_value='/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml',
            description='Face detection cascade filename'
        ),
        DeclareLaunchArgument(
            'eyes_cascade_name',
            default_value='/usr/share/opencv4/haarcascades/haarcascade_eye_tree_eyeglasses.xml',
            description='Eye detection cascade filename'
        ),

        # Node
        Node(
            package='opencv_apps',
            executable='face_detection',
            name=LaunchConfiguration('node_name'),
            remappings=[
                ('image', LaunchConfiguration('image')),
            ],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'face_cascade_name': LaunchConfiguration('face_cascade_name'),
                'eyes_cascade_name': LaunchConfiguration('eyes_cascade_name'),
            }]
        ),
    ])
