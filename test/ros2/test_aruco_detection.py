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


# test of aruco_detection

import unittest

from geometry_msgs.msg import PoseArray
from opencv_apps.msg import DetectionArrayStamped
from opencv_apps_test import camera, CODES_VIDEO, description, image_view, include, MessagesTest
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(CODES_VIDEO),
        include('aruco_detection', image='image', use_camera_info='true', marker_length='0.05'),
        image_view('aruco_detection/image', name='aruco_detection_view'),
    )


class TestArucoDetection(MessagesTest, unittest.TestCase):

    TOPICS = ['aruco_detection/detections', 'aruco_detection/poses', 'aruco_detection/image']
    # the markers 1 and 7 of test/make_codes_video.py, in front of the camera
    CHECKS = {
        'aruco_detection/detections': (DetectionArrayStamped, lambda msg: sorted(
            d.id for d in msg.detections) == [1, 7] and all(d.label == 'DICT_4X4_50' for d in msg.detections)),
        'aruco_detection/poses': (PoseArray, lambda msg: len(msg.poses) == 2 and all(
            p.position.z > 0 for p in msg.poses)),
    }
