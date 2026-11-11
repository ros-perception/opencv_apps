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

# Run all opencv_apps nodes with a synthetic camera, and check that each of them publishes its output
# (nodes subscribe the input only while their outputs are subscribed).

import os
import time
import unittest

import cv2
import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from opencv_apps.msg import Point2D, Point2DArray
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

NODES = [
    'adding_images', 'blob_detection', 'camshift', 'contour_moments', 'convex_hull', 'corner_harris',
    'discrete_fourier_transform', 'edge_detection', 'equalize_histogram', 'face_detection', 'fback_flow',
    'find_contours', 'general_contours', 'goodfeature_track', 'hls_color_filter', 'hough_circles', 'hough_lines',
    'hsv_color_filter', 'lab_color_filter', 'lk_flow', 'morphology', 'people_detect', 'phase_corr', 'pyramids',
    'rgb_color_filter', 'segment_objects', 'simple_flow', 'smoothing', 'threshold', 'watershed_segmentation',
]
if os.environ.get('OPENCV_APPS_HAVE_OPTFLOW', '1') == '0':  # simple_flow needs opencv_contrib's optflow
    NODES.remove('simple_flow')
OUTPUTS = ['/%s/image' % name for name in NODES] + ['/image_converter/output_video/raw',
                                                     '/image_converter/output_video/compressed']


@pytest.mark.launch_test
def generate_test_description():
    nodes = []
    for name in NODES:
        nodes.append(launch_ros.actions.Node(
            package='opencv_apps', executable=name, name=name, output='screen',
            remappings=[('image', '/camera/image'), ('camera_info', '/camera/camera_info'),
                        ('image1', '/camera/image'), ('image2', '/camera/image')]))
    for name in ['simple_example', 'simple_compressed_example']:
        nodes.append(launch_ros.actions.Node(
            package='opencv_apps', executable=name, name=name, output='screen',
            remappings=[('image', '/camera/image'), ('image/compressed', '/camera/image/compressed')]))
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()])


class TestNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_opencv_apps_nodes')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def camera_image(self, count):
        img = np.full((240, 320, 3), 64, np.uint8)
        x = 40 + (count * 2) % 80
        cv2.rectangle(img, (x, 60), (x + 100, 160), (0, 0, 255), -1)
        cv2.circle(img, (240, 160), 40, (0, 255, 0), -1)
        cv2.line(img, (0, 220), (320, 200), (255, 255, 255), 3)
        return img

    def test_outputs(self):
        node = self.node
        received = {}
        for topic in OUTPUTS:
            msg_type = CompressedImage if topic.endswith('compressed') else Image
            node.create_subscription(msg_type, topic,
                                     lambda msg, topic=topic: received.__setitem__(topic, True), 10)
        image_pub = node.create_publisher(Image, '/camera/image', 10)
        info_pub = node.create_publisher(CameraInfo, '/camera/camera_info', 10)
        compressed_pub = node.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        # watershed_segmentation needs seed points
        seed_pub = node.create_publisher(Point2DArray, '/watershed_segmentation/add_seed_points', 10)
        seeds = Point2DArray(points=[Point2D(x=10.0, y=10.0), Point2D(x=240.0, y=160.0), Point2D(x=100.0, y=100.0)])

        count = 0
        end_time = time.time() + 60
        while time.time() < end_time and len(received) < len(OUTPUTS):
            img = self.camera_image(count)
            stamp = node.get_clock().now().to_msg()
            msg = Image(height=img.shape[0], width=img.shape[1], encoding='bgr8', step=img.shape[1] * 3,
                        data=img.tobytes())
            msg.header.stamp = stamp
            msg.header.frame_id = 'camera'
            info = CameraInfo(height=img.shape[0], width=img.shape[1])
            info.header = msg.header
            compressed = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', img)[1].tobytes())
            compressed.header = msg.header
            image_pub.publish(msg)
            info_pub.publish(info)
            compressed_pub.publish(compressed)
            seed_pub.publish(seeds)
            rclpy.spin_once(node, timeout_sec=0.1)
            count += 1
        self.assertEqual(sorted(set(OUTPUTS) - set(received)), [])
