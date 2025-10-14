#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='camshift',
        description='Name of the node'
    )

    image_arg = DeclareLaunchArgument(
        'image',
        default_value='image',
        description='The image topic. Should be remapped to the name of the real image topic.'
    )

    use_camera_info_arg = DeclareLaunchArgument(
        'use_camera_info',
        default_value='false',
        description='Indicates that the camera_info topic should be subscribed to'
    )

    debug_view_arg = DeclareLaunchArgument(
        'debug_view',
        default_value='true',
        description='Specify whether the node displays a window to show edge image'
    )

    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='3',
        description='Specify queue_size of input image subscribers'
    )

    histogram_arg = DeclareLaunchArgument(
        'histogram',
        default_value='[]',
        description='Histogram of tracked color object'
    )

    vmin_arg = DeclareLaunchArgument(
        'vmin',
        default_value='10',
        description='Min threshold of lightness'
    )

    vmax_arg = DeclareLaunchArgument(
        'vmax',
        default_value='256',
        description='Max threshold of lightness'
    )

    smin_arg = DeclareLaunchArgument(
        'smin',
        default_value='30',
        description='Min value of saturation'
    )

    always_subscribe_arg = DeclareLaunchArgument(
        'always_subscribe',
        default_value='false',
        description='Always subscribe to input topics regardless of output subscribers'
    )

    # Create the node
    camshift_node = Node(
        package='opencv_apps',
        executable='camshift',
        name=LaunchConfiguration('node_name'),
        remappings=[
            ('image', LaunchConfiguration('image')),
        ],
        parameters=[{
            'use_camera_info': LaunchConfiguration('use_camera_info'),
            'debug_view': LaunchConfiguration('debug_view'),
            'queue_size': LaunchConfiguration('queue_size'),
            'vmin': LaunchConfiguration('vmin'),
            'vmax': LaunchConfiguration('vmax'),
            'smin': LaunchConfiguration('smin'),
            'always_subscribe': LaunchConfiguration('always_subscribe'),
        }],
        output='screen',
    )

    return LaunchDescription([
        node_name_arg,
        image_arg,
        use_camera_info_arg,
        debug_view_arg,
        queue_size_arg,
        histogram_arg,
        vmin_arg,
        vmax_arg,
        smin_arg,
        always_subscribe_arg,
        camshift_node,
    ])
