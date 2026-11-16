# Software License Agreement (BSD License)
#
# Copyright (c) 2026, Kei Okada.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Kei Okada nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# The odometry of opencv_apps on TurtleBot3 Waffle of the simulation of Nav2 (Gazebo, nav2_minimal_tb3_sim), instead
# of the odometry of its wheels: laser_odometry of its 2D LiDAR (scan) and rgbd_odometry of its depth camera (icp,
# without the image), one of which (odometry) publishes odom -> base_footprint and /odom for slam_toolbox (the 2D map)
# and Nav2, and the other only its trajectory and its 3D map (~/map). ROS 2 only; needs nav2_bringup,
# nav2_minimal_tb3_sim and slam_toolbox (sudo apt install ros-$ROS_DISTRO-nav2-bringup
# ros-$ROS_DISTRO-nav2-minimal-tb3-sim ros-$ROS_DISTRO-slam-toolbox).
#   ros2 launch opencv_apps odometry_tb3_simulation.launch.py [odometry:=laser|camera] [headless:=False]
#   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share opencv_apps)/rviz/odometry_tb3_simulation.rviz
# and send a goal by Nav2 Goal of rviz.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def robot_sdf(sim_dir):
    """Return the SDF of the robot, whose depth camera is on camera/depth/image of its optical frame, at 10 Hz."""
    sdf = xacro.process_file(os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'), mappings={'namespace': ''})
    sdf = sdf.toxml()
    start = sdf.index('<sensor name="intel_realsense_r200_depth"')
    end = sdf.index('</sensor>', start)
    sensor = sdf[start:end]
    sensor = sensor.replace('<gz_frame_id>camera_depth_frame</gz_frame_id>',
                            '<gz_frame_id>camera_depth_optical_frame</gz_frame_id>')
    sensor = sensor.replace('<update_rate>5</update_rate>', '<update_rate>10</update_rate>')
    sensor = sensor.replace('<always_on>1</always_on>', '<always_on>1</always_on><topic>camera/depth/image</topic>')
    return sdf[:start] + sensor + sdf[end:]


def launch_setup(context):
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    share = get_package_share_directory('opencv_apps')
    odometry = LaunchConfiguration('odometry').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    world = os.path.join(tempfile.mkdtemp(prefix='opencv_apps_'), 'tb3_sandbox.sdf')
    with open(world, 'w') as f:
        f.write(xacro.process_file(os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
                                   mappings={'headless': headless}).toxml())
    with open(os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')) as f:
        urdf = f.read()
    sim_time = {'use_sim_time': True}
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r'] + (['-s'] if headless.lower() == 'true' else []) + [world],
                       output='screen'),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-string', robot_sdf(sim_dir), '-name', 'turtlebot3_waffle', '-x', '-2.0', '-y', '-0.5',
                        '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'tb3_bridge.yaml')}, sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}, sim_time]),
        # the odometry of the 2D LiDAR and of the depth camera; one of them for slam_toolbox and Nav2
        Node(package='opencv_apps', executable='laser_odometry', name='laser_odometry', output='screen',
             parameters=[{'input_type': 'scan', 'base_frame': 'base_footprint', 'voxel_size': 0.05,
                          'max_range': 3.5, 'min_range': 0.12, 'correspondence_distance': 0.5,
                          'map_voxel_size': 0.05, 'publish_tf': odometry == 'laser'}, sim_time],
             remappings=[('~/odom', '/odom')] if odometry == 'laser' else []),
        Node(package='opencv_apps', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
             parameters=[{'use_image': False, 'method': 'icp', 'base_frame': 'base_footprint', 'max_depth': 5.0,
                          'map_voxel_size': 0.05, 'publish_tf': odometry == 'camera'}, sim_time],
             remappings=[('depth', 'camera/depth/image'), ('camera_info', 'camera/depth/camera_info')] +
             ([('~/odom', '/odom')] if odometry == 'camera' else [])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch',
                                                       'online_async_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(share, 'config', 'tb3_slam.yaml'),
                              'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                                                       'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odometry', default_value='laser',
                              description='The odometry for slam_toolbox and Nav2: laser (laser_odometry of the 2D '
                                          'LiDAR) or camera (rgbd_odometry of the depth camera)'),
        DeclareLaunchArgument('headless', default_value='True', description='Without the GUI of Gazebo'),
        OpaqueFunction(function=launch_setup),
    ])
