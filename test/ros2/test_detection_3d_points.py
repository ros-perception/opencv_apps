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


# 3D boxes and TF frames of the people and the vehicles detected by dnn_object_detection in the images of
# test/lidar_boreas.mp4, with the points of the LiDAR in test/lidar_boreas_points.npz (Boreas), published by
# test/publish_lidar.py with the static TF of the extrinsics

import os
import sys
import unittest

import launch.actions
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

LABELS = ('person', 'car', 'truck', 'bus')


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_lidar.py')],
                                      output='screen'),
        include('dnn_object_detection', image='image'),
        include('detection_3d', input='dnn_object_detection/detections', points='points', camera_info='camera_info',
                depth_source='points', max_depth='60.0', depth_threshold='1.0', track_distance='2.0'),
    )


def object_3d(detection):
    # a person or a vehicle on the road, 3 - 50 m in front of the camera
    box = detection.bbox
    return (any(r.hypothesis.class_id in LABELS for r in detection.results) and
            3.0 < box.center.position.z < 50.0 and 0.0 < box.size.x < 10.0 and 0.0 < box.size.y < 10.0)


def object_frame(transform):
    return (transform.child_frame_id.split('_')[0] in LABELS and transform.header.frame_id == 'camera' and
            3.0 < transform.transform.translation.z < 50.0)


class TestDetection3DPoints(MessagesTest, unittest.TestCase):

    TOPICS = ['detection_3d/detections']
    CHECKS = {
        'detection_3d/detections': (Detection3DArray, lambda msg: msg.header.frame_id == 'camera' and
                                    any(object_3d(d) for d in msg.detections)),
        'tf': (TFMessage, lambda msg: any(object_frame(t) for t in msg.transforms)),
    }
