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


# Common parts of the ROS2 tests test/ros2/test_*.py, the counterparts of the ROS1 test/test-*.test files: the
# input images come from a video file played by image_publisher (instead of rosbag play and image_proc), and
# TopicsTest checks that each of the TOPICS receives messages (instead of hztest).
# Run with gui:=true (e.g. `launch_test test/ros2/test_edge_detection.py gui:=true`) to see the debug_view windows.

import os
import tarfile
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_testing.actions
import numpy as np
import rclpy
from rosidl_runtime_py.utilities import get_message

TEST_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# wide_stereo/left of face_detector_withface_test_diamondback.bag, used by most of the ROS1 tests
FACE_VIDEO = os.path.join(TEST_DIR, 'face_detector_withface_test_diamondback.mp4')
# narrow_stereo/left/image_rect (5s - 15s) of vslam_tutorial.bag, used by the optical flow tests
VSLAM_VIDEO = os.path.join(TEST_DIR, 'vslam_tutorial.mp4')
# a QR code, a barcode and ArUco markers, made by test/make_codes_video.py
CODES_VIDEO = os.path.join(TEST_DIR, 'codes.mp4')
# words on a sheet, made by test/make_text_video.py
TEXT_VIDEO = os.path.join(TEST_DIR, 'text.mp4')
GUI = LaunchConfiguration('gui')

# Fast DDS profile with the shared memory large enough for the images, for this process and the nodes it launches,
# see fastdds.xml (FASTRTPS_ for Fast DDS 2 of jazzy, FASTDDS_ for Fast DDS 3)
for _variable in ['FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE']:
    os.environ.setdefault(_variable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fastdds.xml'))


def face_data_dir():
    """Extract test/face_data.tar.gz (the trained data of face_recognition) to a temporary directory."""
    path = tempfile.mkdtemp(prefix='opencv_apps_test_')
    with tarfile.open(os.path.join(TEST_DIR, 'face_data.tar.gz')) as tar:
        tar.extractall(path)
    return os.path.join(path, 'face_data')


def camera(video, rate=10.0):
    """Publish the video on image (and camera_info) in a loop."""
    return launch_ros.actions.Node(
        package='image_publisher', executable='image_publisher_node', name='camera', arguments=[video],
        parameters=[{'publish_rate': rate}], remappings=[('image_raw', 'image')])


def include(name, **args):
    """Include launch/<name>.launch.xml, with debug_view:=$(var gui).

    The include is scoped, otherwise its arguments (e.g. node_name) would also be set in the following includes.
    """
    path = os.path.join(get_package_share_directory('opencv_apps'), 'launch', name + '.launch.xml')
    return GroupAction([IncludeLaunchDescription(
        AnyLaunchDescriptionSource(path),
        launch_arguments=[('debug_view', GUI)] + [(k, str(v)) for k, v in args.items()])], scoped=True)


def image_view(topic, name='image_view'):
    return launch_ros.actions.Node(package='image_view', executable='image_view', name=name,
                                   remappings=[('image', topic)], condition=IfCondition(GUI))


def description(*actions):
    return launch.LaunchDescription([DeclareLaunchArgument('gui', default_value='false')] + list(actions) +
                                    [launch_testing.actions.ReadyToTest()])


class TopicsTest:
    """Mixin of unittest.TestCase: check that each of TOPICS receives MIN_MESSAGES messages within TIMEOUT."""

    TOPICS = []
    MIN_MESSAGES = 5
    TIMEOUT = 120.0

    def prepare(self, node):
        """Override to publish or call services that the nodes under test need."""

    def check_message(self, topic, msg):
        """Override to check the content of the last message received on topic."""

    def test_topics(self):
        rclpy.init()
        try:
            node = rclpy.create_node('test_' + self.__class__.__name__.lower())
            self.prepare(node)
            topics = [t if t.startswith('/') else '/' + t for t in self.TOPICS]
            counts = {}
            subscriptions = []
            messages = {}

            def callback(msg, topic):
                counts[topic] += 1
                messages[topic] = msg

            end_time = time.time() + self.TIMEOUT
            while time.time() < end_time and \
                    (len(counts) < len(topics) or min(counts.values()) < self.MIN_MESSAGES):
                # subscribe each topic with its type, once it is advertised
                types = dict(node.get_topic_names_and_types())
                for topic in topics:
                    if topic not in counts and topic in types:
                        counts[topic] = 0
                        subscriptions.append(node.create_subscription(
                            get_message(types[topic][0]), topic,
                            lambda msg, topic=topic: callback(msg, topic), 10))
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertEqual({t: counts.get(t, 0) for t in topics if counts.get(t, 0) < self.MIN_MESSAGES}, {})
            for topic in topics:
                self.check_message(topic, messages[topic])
        finally:
            rclpy.shutdown()


class ColorSpaceTest(TopicsTest):
    """TopicsTest that also checks the ~color_space PointCloud2 of the color filters."""

    def check_message(self, topic, msg):
        if not topic.endswith('/color_space'):
            return
        float32 = 7  # sensor_msgs/PointField FLOAT32
        self.assertEqual([(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
                         [('x', 0, float32, 1), ('y', 4, float32, 1), ('z', 8, float32, 1), ('rgb', 12, float32, 1)])
        self.assertEqual(msg.point_step, 16)
        self.assertEqual(msg.row_step, msg.width * msg.point_step)
        self.assertGreater(msg.width * msg.height, 0)
        self.assertEqual(len(msg.data), msg.height * msg.row_step)
        self.assertNotEqual(msg.header.frame_id, '')
        xyz = np.frombuffer(bytes(msg.data), np.float32).reshape(-1, 4)[:, :3]
        self.assertTrue(np.all((0.0 <= xyz) & (xyz <= 1.0)), 'x, y, z of %s are not within 0..1' % topic)


class MessagesTest(TopicsTest):
    """TopicsTest, and check that each topic of CHECKS receives a message that its function returns True for."""

    # topic: (message type, function of the message)
    CHECKS = {}

    def test_messages(self):
        rclpy.init()
        try:
            node = rclpy.create_node('test_messages_' + self.__class__.__name__.lower())
            self.prepare(node)
            found = set()
            subscriptions = []
            for topic, (msg_type, check) in self.CHECKS.items():
                topic = topic if topic.startswith('/') else '/' + topic
                subscriptions.append(node.create_subscription(
                    msg_type, topic, lambda msg, topic=topic, check=check: check(msg) and found.add(topic), 10))
            end_time = time.time() + self.TIMEOUT
            while time.time() < end_time and len(found) < len(self.CHECKS):
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertEqual(sorted(t for t in self.CHECKS if (t if t.startswith('/') else '/' + t) not in found), [])
        finally:
            rclpy.shutdown()
