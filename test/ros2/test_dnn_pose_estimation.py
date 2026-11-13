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


# test of dnn_pose_estimation (OpenCV 4.9 or later)

import unittest

from opencv_apps.msg import DetectionArrayStamped
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, MessagesTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        include('dnn_pose_estimation', image='image'),
        image_view('dnn_pose_estimation/image', name='dnn_pose_estimation_view'),
    )


def has_pose(msg):
    # a person with the 33 landmarks, the nose (0) above the shoulders (11, 12) and the shoulders above the hips (23, 24)
    return any(d.label == 'person' and d.score > 0.5 and len(d.corners) == 33 and
               d.corners[0].y < min(d.corners[11].y, d.corners[12].y) and
               max(d.corners[11].y, d.corners[12].y) < min(d.corners[23].y, d.corners[24].y) for d in msg.detections)


class TestDNNPoseEstimation(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_pose_estimation/detections', 'dnn_pose_estimation/image']
    CHECKS = {'dnn_pose_estimation/detections': (DetectionArrayStamped, has_pose)}
