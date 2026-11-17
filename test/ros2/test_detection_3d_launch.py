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


# detection_3d:=true of launch/dnn_object_detection.launch.xml: the 3D boxes of the people of dnn_object_detection,
# by detection_3d of launch/detection_3d_pipeline.launch.xml, with the depth (full name) and the camera_info next to the
# image of test/publish_rgbd.py, in the namespace camera1, where the topics of the pipeline are

import os
import sys
import unittest

import launch.actions
import launch_ros.actions
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest
from test_detection_3d import box_edges, person
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_rgbd.py')], output='screen'),
        launch.actions.GroupAction([
            launch_ros.actions.PushRosNamespace('camera1'),
            include('dnn_object_detection', image='/image', detection_3d='true', depth='/depth'),
        ]),
    )


class TestDetection3DLaunch(MessagesTest, unittest.TestCase):

    TOPICS = ['camera1/dnn_object_detection_3d/detections']
    CHECKS = {
        'camera1/dnn_object_detection_3d/detections': (
            Detection3DArray, lambda msg: msg.header.frame_id == 'camera_rgb_optical_frame' and
            any(person(d) for d in msg.detections)),
        'camera1/dnn_object_detection_3d/markers': (MarkerArray, lambda msg: any(box_edges(m) for m in msg.markers)),
    }
