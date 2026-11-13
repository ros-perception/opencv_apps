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


# test of the tracking of dnn_hand_pose_estimation (OpenCV 4.9 or later), with the frames of the face video
# published one by one: the hands (of the woman holding a phone) keep their ids while they are found, see also
# test_dnn_pose_estimation_tracking.py

import unittest

import numpy as np
from opencv_apps.msg import SkeletonArrayStamped
from opencv_apps_test import description, FACE_VIDEO, include, publish_frames, video_frames
import pytest
import rclpy

TOPIC = 'dnn_hand_pose_estimation/skeletons'


@pytest.mark.launch_test
def generate_test_description():
    return description(include('dnn_hand_pose_estimation', image='image'))


def center(skeleton):
    return np.mean([[k.x, k.y] for k in skeleton.keypoints], axis=0)


class TestDNNHandPoseEstimationTracking(unittest.TestCase):

    def test_tracking(self):
        frames = video_frames(FACE_VIDEO, 0, 198)
        rclpy.init()
        try:
            node = rclpy.create_node('test_dnn_hand_pose_estimation_tracking')
            # the first image waits for the models, downloaded at the first time
            results = publish_frames(node, frames, {TOPIC: SkeletonArrayStamped}, timeout=120.0)
        finally:
            rclpy.shutdown()
        self.assertTrue(all(r[TOPIC] is not None for r in results), 'no output of some images')
        hands = [r[TOPIC].skeletons for r in results]
        print('ids of the hands: %s' % ' '.join(','.join(str(h.id) for h in hs) or '-' for hs in hands))
        # the hands found in the consecutive images at the same place (within 30 pixels) keep their ids
        same, kept = 0, 0
        for previous, current in zip(hands, hands[1:]):
            for hand in current:
                near = [p for p in previous if np.linalg.norm(center(p) - center(hand)) < 30]
                if near:
                    same += 1
                    kept += any(p.id == hand.id for p in near)
        print('found in %d images, %d of %d hands keep their ids' % (sum(bool(h) for h in hands), kept, same))
        # the hands are small (about 34 pixels), found in about 26 of the 199 images
        self.assertGreater(sum(bool(h) for h in hands), 20)
        self.assertGreater(kept, 0.9 * same)
