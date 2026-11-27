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


# ROS2 version of test/test-pyramids.test

import unittest

import launch_ros.actions
from opencv_apps_test import camera, description, FACE_VIDEO, GUI, TopicsTest
import pytest
import rclpy
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


@pytest.mark.launch_test
def generate_test_description():
    # the chain of the nodes on the images, which subscribe them with reliable_subscription
    return description(
        camera(FACE_VIDEO),
        launch_ros.actions.Node(
            package='opencv_apps', executable='pyramids', name='pyramids_zoomin',
            parameters=[{'pyramids_type': 0, 'num_of_pyramids': 1, 'debug_view': GUI, 'reliable_subscription': True}],
            remappings=[('image', 'image')]),
        launch_ros.actions.Node(
            package='opencv_apps', executable='adding_images', name='adding_images',
            parameters=[{'alpha': 0.0, 'debug_view': GUI, 'reliable_subscription': True}],
            remappings=[('image1', 'pyramids_zoomin/image'), ('image2', 'image')]),
    )


class TestReliableSubscription(TopicsTest, unittest.TestCase):

    TOPICS = ['pyramids_zoomin/image', 'adding_images/image']

    def test_qos(self):
        # the subscriptions of the images by the nodes are reliable
        rclpy.init()
        try:
            node = rclpy.create_node('test_reliable_subscription_qos')
            # the nodes subscribe the images only with subscribers of their outputs
            node.create_subscription(Image, 'adding_images/image', lambda msg: None, 1)
            expected = {'/image': 'pyramids_zoomin', '/pyramids_zoomin/image': 'adding_images'}
            found = {}
            for _ in range(600):
                for topic in expected:
                    for info in node.get_subscriptions_info_by_topic(topic):
                        if info.node_name == expected[topic]:
                            found[topic] = info.qos_profile.reliability
                if len(found) == len(expected):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertEqual(found, {topic: ReliabilityPolicy.RELIABLE for topic in expected})
        finally:
            rclpy.shutdown()
