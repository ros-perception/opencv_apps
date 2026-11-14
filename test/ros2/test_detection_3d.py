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


# 3D boxes and TF frames of the people detected by dnn_object_detection in the RGB-D images of test/rgbd_walking.mp4
# and test/rgbd_walking_depth.npz (TUM RGB-D), published by test/publish_rgbd.py

import os
import sys
import unittest

import launch.actions
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_rgbd.py')], output='screen'),
        include('dnn_object_detection', image='image'),
        include('detection_3d', input='dnn_object_detection/detections', depth='depth', camera_info='camera_info'),
    )


def person(detection):
    # the person walks about 1.5 m in front of the camera
    box = detection.bbox
    return (any(r.hypothesis.class_id == 'person' and r.hypothesis.score > 0.5 for r in detection.results) and
            detection.id.startswith('person_') and 0.8 < box.center.position.z < 3.0 and
            0.2 < box.size.x < 1.5 and 0.5 < box.size.y < 2.5)


def person_frame(transform):
    # the TF frame of the person, in the camera frame
    return (transform.child_frame_id == 'person_0' and transform.header.frame_id == 'camera_rgb_optical_frame' and
            0.8 < transform.transform.translation.z < 3.0)


class TestDetection3D(MessagesTest, unittest.TestCase):

    TOPICS = ['detection_3d/detections']
    CHECKS = {
        'detection_3d/detections': (Detection3DArray, lambda msg: msg.header.frame_id == 'camera_rgb_optical_frame' and
                                    any(person(d) for d in msg.detections)),
        'tf': (TFMessage, lambda msg: any(person_frame(t) for t in msg.transforms)),
    }
