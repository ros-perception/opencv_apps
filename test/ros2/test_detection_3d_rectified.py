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

# detection_3d with a rectified camera whose R is not the identity and P is not K (test/publish_rectified.py):
# the square at OBJECT of the optical frame, by the depth image and by the points (with the padding of the rows), and
# the TF frames of the keypoint nose at its center, in the optical frame (not the rectified frame)

import os
import sys
import unittest

import launch.actions
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

OBJECT = (0.3, -0.1, 2.0)  # of test/publish_rectified.py


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_rectified.py')],
                                      output='screen'),
        include('detection_3d', input='skeletons', depth='depth', camera_info='camera_info',
                input_type='skeletons', rectified='true'),
        include('detection_3d', node_name='detection_3d_points', input='skeletons', points='points',
                camera_info='camera_info', input_type='skeletons', depth_source='points', rectified='true',
                tf_prefix='points_'),
    )


def near(position, tolerance=0.03):
    return all(abs(a - b) < tolerance for a, b in zip((position.x, position.y, position.z), OBJECT))


def at_object(msg):
    return msg.header.frame_id == 'camera' and any(near(d.bbox.center.position) for d in msg.detections)


frames = set()


def keypoint_frames(msg):
    # the keypoints of the both nodes, at the object
    frames.update(t.child_frame_id for t in msg.transforms
                  if t.child_frame_id in ('person_0_nose', 'points_person_0_nose') and near(t.transform.translation))
    return len(frames) == 2


class TestDetection3DRectified(MessagesTest, unittest.TestCase):

    TOPICS = ['detection_3d/detections', 'detection_3d_points/detections']
    CHECKS = {
        'detection_3d/detections': (Detection3DArray, at_object),
        'detection_3d_points/detections': (Detection3DArray, at_object),
        'tf': (TFMessage, keypoint_frames),
    }
