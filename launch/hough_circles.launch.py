from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'node_name',
            default_value='hough_circles',
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
            'canny_threshold',
            default_value='200',
            description='Upper threshold for the internal Canny edge detector'
        ),
        DeclareLaunchArgument(
            'accumulator_threshold',
            default_value='50',
            description='Threshold for center detection'
        ),
        DeclareLaunchArgument(
            'gaussian_blur_size',
            default_value='9',
            description='The size of gaussian blur (should be odd number)'
        ),
        DeclareLaunchArgument(
            'gaussian_sigma_x',
            default_value='2',
            description='Sigma x of gaussian kernel'
        ),
        DeclareLaunchArgument(
            'gaussian_sigma_y',
            default_value='2',
            description='Sigma y of gaussian kernel'
        ),
        DeclareLaunchArgument(
            'dp',
            default_value='2',
            description='The inverse ratio of resolution'
        ),
        DeclareLaunchArgument(
            'min_circle_radius',
            default_value='0',
            description='The minimum size of the circle, If unknown, put zero as default'
        ),
        DeclareLaunchArgument(
            'max_circle_radius',
            default_value='0',
            description='The maximum size of the circle, If unknown, put zero as default'
        ),

        # Node
        Node(
            package='opencv_apps',
            executable='hough_circles',
            name=LaunchConfiguration('node_name'),
            remappings=[
                ('image', LaunchConfiguration('image')),
            ],
            parameters=[{
                'use_camera_info': LaunchConfiguration('use_camera_info'),
                'debug_view': LaunchConfiguration('debug_view'),
                'queue_size': LaunchConfiguration('queue_size'),
                'canny_threshold': LaunchConfiguration('canny_threshold'),
                'accumulator_threshold': LaunchConfiguration('accumulator_threshold'),
                'gaussian_blur_size': LaunchConfiguration('gaussian_blur_size'),
                'gaussian_sigma_x': LaunchConfiguration('gaussian_sigma_x'),
                'gaussian_sigma_y': LaunchConfiguration('gaussian_sigma_y'),
                'dp': LaunchConfiguration('dp'),
                'min_circle_radius': LaunchConfiguration('min_circle_radius'),
                'max_circle_radius': LaunchConfiguration('max_circle_radius'),
            }]
        ),
    ])
