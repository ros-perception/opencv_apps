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


# ROS2 version of test/test-tracking.test

import os
import unittest

import launch_ros.actions
from opencv_apps.msg import RectArrayStamped
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import pytest

# KCF tracker is in opencv_contrib's tracking module, use MIL tracker of video module without it
TRACKING_ALGORITHM = 4 if os.environ.get('OPENCV_APPS_HAVE_TRACKING', '1') == '1' else 0


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        image_view('image', name='image_view'),
        include('tracking', image='image', tracking_algorithm=TRACKING_ALGORITHM),
        # track the face of the person sitting at the center, from the next input image
        launch_ros.actions.Node(package='opencv_apps', executable='set_image.py', name='tracking_set_roi',
                                arguments=['377', '211', '50', '50'], remappings=[('set_roi', 'tracking/set_roi')]),
    )


class TestTracking(TopicsTest, unittest.TestCase):

    TOPICS = ['tracking/image', 'tracking/output']
    MIN_MESSAGES = 30

    def prepare(self, node):
        # rects of tracking/output around the face
        self.tracked = []
        node.create_subscription(RectArrayStamped, 'tracking/output', lambda msg: self.tracked.extend(
            r for r in msg.rects if abs(r.x - 377) < 50 and abs(r.y - 211) < 50), 10)

    def test_topics(self):
        super(TestTracking, self).test_topics()
        # the first one is published at the initialization by set_roi, so the others are tracked
        self.assertGreaterEqual(len(self.tracked), 2)
