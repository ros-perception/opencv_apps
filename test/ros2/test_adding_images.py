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


# ROS2 version of test/test-adding-images.test

import unittest

import launch_ros.actions
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        image_view('image', name='image_view0'),
        image_view('hough_lines/image', name='image_view1'),
        launch_ros.actions.Node(
            package='opencv_apps', executable='hough_circles', name='hough_circles'),
        include('adding_images', image1='image', image2='hough_circles/image', alpha='0.5', gamma='0.0',
                approximate_sync='false'),
        launch_ros.actions.Node(
            package='image_proc', executable='resize_node', name='resize',
            parameters=[{'scale_height': 0.5, 'scale_width': 0.5}],
            remappings=[('image/image_raw', 'image'), ('image/camera_info', 'camera_info'),
                        ('resize/image_raw', 'resize/image')]),
        include('adding_images', image1='image', image2='resize/image', alpha='0.5', gamma='0.0',
                approximate_sync='false', node_name='adding_images_resize'),
    )


class TestAddingImages(TopicsTest, unittest.TestCase):

    TOPICS = ['adding_images/image', 'adding_images_resize/image']
