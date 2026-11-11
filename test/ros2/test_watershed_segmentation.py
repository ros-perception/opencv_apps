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

import unittest

from opencv_apps.msg import Point2D, Point2DArray
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        image_view('image', name='image_view'),
        include('watershed_segmentation', use_camera_info='false', image='image'),
    )


class TestWatershedSegmentation(TopicsTest, unittest.TestCase):

    TOPICS = ['watershed_segmentation/contours']

    def prepare(self, node):
        # rostopic pub -r 1.5 watershed_segmentation/add_seed_points
        pub = node.create_publisher(Point2DArray, 'watershed_segmentation/add_seed_points', 1)
        points = Point2DArray(points=[Point2D(x=float(x), y=float(y)) for x, y in [
            (100, 100), (200, 100), (300, 100), (400, 100), (500, 100),
            (100, 200), (200, 200), (300, 200), (400, 200), (500, 200),
            (100, 300), (200, 300), (300, 300), (400, 300), (500, 300)]])
        self.timer = node.create_timer(1 / 1.5, lambda: pub.publish(points))
