from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'node_name',
            default_value='edge_detection',
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
            'edge_type',
            default_value='0',
            description='Specify edge detection methods. 0: Sobel Derivatives, 1: Laplace Operator, 2: Canny Edge Detector'
        ),
        DeclareLaunchArgument(
            'canny_threshold1',
            default_value='100',
            description='Specify second canny threshold value'
        ),
        DeclareLaunchArgument(
            'canny_threshold2',
            default_value='200',
            description='Specify first canny threshold value'
        ),
        DeclareLaunchArgument(
            'apertureSize',
            default_value='3',
            description='Aperture size for the Sobel() operator'
        ),
        DeclareLaunchArgument(
            'apply_blur_pre',
            default_value='true',
            description='Flag, applying Blur() to input image'
        ),
        DeclareLaunchArgument(
            'postBlurSize',
            default_value='13',
            description='Aperture size for the Blur() to input image'
        ),
        DeclareLaunchArgument(
            'postBlurSigma',
            default_value='3.2',
            description='Sigma for the GaussianBlur() to input image'
        ),
        DeclareLaunchArgument(
            'apply_blur_post',
            default_value='false',
            description='Flag, applying GaussianBlur() to output(edge) image'
        ),
        DeclareLaunchArgument(
            'L2gradient',
            default_value='false',
            description='Flag, L2Gradient'
        ),

        # Node
        Node(
            package='opencv_apps',
            executable='edge_detection',
            name=LaunchConfiguration('node_name'),
            remappings=[
                ('image', LaunchConfiguration('image')),
            ],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'edge_type': LaunchConfiguration('edge_type'),
                'canny_threshold1': LaunchConfiguration('canny_threshold1'),
                'canny_threshold2': LaunchConfiguration('canny_threshold2'),
                'apertureSize': LaunchConfiguration('apertureSize'),
                'apply_blur_pre': LaunchConfiguration('apply_blur_pre'),
                'postBlurSize': LaunchConfiguration('postBlurSize'),
                'postBlurSigma': LaunchConfiguration('postBlurSigma'),
                'apply_blur_post': LaunchConfiguration('apply_blur_post'),
                'L2gradient': LaunchConfiguration('L2gradient'),
            }]
        ),
    ])
