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


# test of dnn_hand_pose_estimation (OpenCV 4.9 or later), with the hands of the persons in the face video, e.g. the
# woman holding a phone

import math
import unittest

from opencv_apps.msg import SkeletonArrayStamped
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, MessagesTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        include('dnn_hand_pose_estimation', image='image'),
        image_view('dnn_hand_pose_estimation/image', name='dnn_hand_pose_estimation_view'),
    )


def has_hand(msg):
    # a left or right hand with the 21 keypoints in the image, the fingers around the wrist in the size of a hand
    for skeleton in msg.skeletons:
        k = {keypoint.name: keypoint for keypoint in skeleton.keypoints}
        if skeleton.label not in ['left', 'right'] or skeleton.score < 0.5 or len(k) != 21:
            continue
        if not all(0 <= p.x < 640 and 0 <= p.y < 480 for p in k.values()):
            continue
        wrist = k['wrist']
        size = max(math.hypot(p.x - wrist.x, p.y - wrist.y) for p in k.values())
        palm = math.hypot(k['middle_finger_mcp'].x - wrist.x, k['middle_finger_mcp'].y - wrist.y)
        if 5 < palm <= size < 200:
            return True
    return False


class TestDNNHandPoseEstimation(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_hand_pose_estimation/skeletons', 'dnn_hand_pose_estimation/image']
    CHECKS = {'dnn_hand_pose_estimation/skeletons': (SkeletonArrayStamped, has_hand)}
