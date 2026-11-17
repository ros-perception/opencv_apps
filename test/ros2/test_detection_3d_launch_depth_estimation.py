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


# detection_3d:=true of launch/dnn_object_detection.launch.xml without depth: the depth estimated from the color image
# by dnn_depth_estimation (Depth Anything V2 Metric, OpenCV 4.10 or later) of launch/detection_3d_pipeline.launch.xml,
# and the 3D boxes of the people by detection_3d with it, in the color images of test/publish_rgbd.py (its depth is
# not used, remapped to unused_depth)

import os
import sys
import unittest

import launch.actions
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest
from sensor_msgs.msg import Image
from test_dnn_depth_estimation_metric import metric
from vision_msgs.msg import Detection3DArray


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_rgbd.py'), '--ros-args',
                                           '-r', 'depth:=unused_depth', '-r', 'points:=unused_points'],
                                      output='screen'),
        include('dnn_object_detection', image='/image', detection_3d='true'),
    )


def person(detection):
    # a person about 1 - 4 m in front of the camera, by the estimated depth (less accurate than that of the camera)
    return (any(r.hypothesis.class_id == 'person' for r in detection.results) and
            0.5 < detection.bbox.center.position.z < 5.0)


class TestDetection3DLaunchDepthEstimation(MessagesTest, unittest.TestCase):

    # the depth estimation on the CPU of the CI is slow (about 1 Hz), within TIMEOUT 180 of the test in CMakeLists.txt
    TIMEOUT = 150.0
    MIN_MESSAGES = 2
    TOPICS = ['dnn_object_detection_depth_estimation/depth', 'dnn_object_detection_3d/detections']
    CHECKS = {
        'dnn_object_detection_depth_estimation/depth': (Image, metric),
        'dnn_object_detection_3d/detections': (Detection3DArray, lambda msg: any(person(d) for d in msg.detections)),
    }
