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



# ROS2 test of dnn_tracking: as test_tracking.py, the face of the person sitting at the center is set by set_image.py
# and tracked, with the newest DNN based tracker of this OpenCV (VitTrack or NanoTrack) and its default model,
# downloaded at the first time.

import unittest

import launch_ros.actions
from opencv_apps.msg import RectArrayStamped
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import pytest


def dnn_tracking_description(tracking_algorithm):
    return description(
        camera(FACE_VIDEO),
        image_view('dnn_tracking/image', name='image_view'),
        include('dnn_tracking', image='image', tracking_algorithm=tracking_algorithm),
        # track the face of the person sitting at the center, from the next input image
        launch_ros.actions.Node(package='opencv_apps', executable='set_image.py', name='dnn_tracking_set_roi',
                                arguments=['377', '211', '50', '50'], remappings=[('set_roi', 'dnn_tracking/set_roi')]),
    )


@pytest.mark.launch_test
def generate_test_description():
    return dnn_tracking_description(-1)


class TestDNNTracking(TopicsTest, unittest.TestCase):

    TOPICS = ['dnn_tracking/image', 'dnn_tracking/output']
    MIN_MESSAGES = 30

    def prepare(self, node):
        # rects of dnn_tracking/output around the face
        self.tracked = []
        node.create_subscription(RectArrayStamped, 'dnn_tracking/output', lambda msg: self.tracked.extend(
            r for r in msg.rects if abs(r.x - 377) < 50 and abs(r.y - 211) < 50), 10)

    def test_topics(self):
        super(TestDNNTracking, self).test_topics()
        # the first one is published at the initialization by set_roi, so the others are tracked
        self.assertGreaterEqual(len(self.tracked), 2)
