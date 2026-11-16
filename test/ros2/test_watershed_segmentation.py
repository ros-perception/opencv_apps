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


# ROS2 version of test/test-watershed-segmentation.test

import time
import unittest

import numpy as np
from opencv_apps.msg import Point2D, Point2DArray
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        image_view('image', name='image_view'),
        include('watershed_segmentation', use_camera_info='false', image='image'),
    )


class TestWatershedSegmentation(TopicsTest, unittest.TestCase):

    TOPICS = ['watershed_segmentation/contours']

    SEEDS = [(100, 100), (200, 100), (300, 100), (400, 100), (500, 100),
             (100, 200), (200, 200), (300, 200), (400, 200), (500, 200),
             (100, 300), (200, 300), (300, 300), (400, 300), (500, 300)]

    def prepare(self, node):
        # rostopic pub -r 1.5 watershed_segmentation/add_seed_points
        pub = node.create_publisher(Point2DArray, 'watershed_segmentation/add_seed_points', 1)
        points = Point2DArray(points=[Point2D(x=float(x), y=float(y)) for x, y in self.SEEDS])
        self.timer = node.create_timer(1 / 1.5, lambda: pub.publish(points))

    def test_colors(self):
        """Check that each region has the same color in every image of ~image.

        ~image is the color of the region * 0.5 + the gray image * 0.5, and the gray image is the same in b, g and r, so
        b - g and g - r of the pixel of a seed point are those of the color of its region.
        """
        rclpy.init()
        try:
            node = rclpy.create_node('test_watershed_segmentation_colors')
            self.prepare(node)
            diffs = []

            def callback(msg):
                image = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
                bgr = image.reshape(msg.height, msg.width, 3).astype(int)
                pixels = np.array([bgr[y, x] for x, y in self.SEEDS])
                diffs.append(np.stack([pixels[:, 0] - pixels[:, 1], pixels[:, 1] - pixels[:, 2]], axis=1))

            node.create_subscription(Image, 'watershed_segmentation/image', callback, 10)
            end_time = time.time() + self.TIMEOUT
            while time.time() < end_time and len(diffs) < 10:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreaterEqual(len(diffs), 10)
            # the first images may be of the seed points of the first messages only
            diffs = np.array(diffs[-5:])
            self.assertLessEqual(int(np.abs(diffs - diffs[0]).max()), 2, 'the colors of the regions change')
        finally:
            rclpy.shutdown()
