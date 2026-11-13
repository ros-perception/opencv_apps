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


# test of the tracking and the smoothing of dnn_pose_estimation (OpenCV 4.9 or later), with the frames of the face
# video published one by one: the woman standing on the right keeps her id, she is found again after the person is
# lost on black images, and the smoothing reduces the jitter of her landmarks

import unittest

import numpy as np
from opencv_apps.msg import SkeletonArrayStamped
from opencv_apps_test import description, FACE_VIDEO, include, publish_frames, video_frames
import pytest
import rclpy

# the frames of the woman, after the man walking in front of her (frames 25 - 39), when their ids may be swapped
FIRST_FRAME, LAST_FRAME = 40, 198
# black images inserted before this frame, to lose her
BLACK_FRAME, BLACK_IMAGES = 120, 5
TOPICS = {'dnn_pose_estimation/skeletons': SkeletonArrayStamped,
          'dnn_pose_estimation_raw/skeletons': SkeletonArrayStamped}


@pytest.mark.launch_test
def generate_test_description():
    return description(
        include('dnn_pose_estimation', image='image'),
        # without the smoothing, to compare the jitter
        include('dnn_pose_estimation', node_name='dnn_pose_estimation_raw', image='image', smoothing='false'),
    )


def woman(msg):
    """Return the skeleton of the woman standing on the right (her nose at x = 370 - 400), or None."""
    for skeleton in msg.skeletons if msg else []:
        if 330 < skeleton.keypoints[0].x < 440:
            return skeleton
    return None


def median_acceleration(skeletons):
    """Return the median of the acceleration (pixels) of the visible keypoints of the consecutive skeletons."""
    accelerations = []
    for s0, s1, s2 in zip(skeletons, skeletons[1:], skeletons[2:]):
        if None in (s0, s1, s2):
            continue
        for k0, k1, k2 in zip(s0.keypoints, s1.keypoints, s2.keypoints):
            if min(k0.visibility, k1.visibility, k2.visibility) > 0.5:
                accelerations.append(np.hypot(k2.x - 2 * k1.x + k0.x, k2.y - 2 * k1.y + k0.y))
    return np.median(accelerations)


class TestDNNPoseEstimationTracking(unittest.TestCase):

    def test_tracking(self):
        frames = video_frames(FACE_VIDEO, FIRST_FRAME, LAST_FRAME)
        black = np.zeros_like(frames[0])
        split = BLACK_FRAME - FIRST_FRAME
        frames = frames[:split] + [black] * BLACK_IMAGES + frames[split:]
        rclpy.init()
        try:
            node = rclpy.create_node('test_dnn_pose_estimation_tracking')
            # the first image waits for the models, downloaded at the first time
            results = publish_frames(node, frames, TOPICS, timeout=120.0, subscribers=2)
        finally:
            rclpy.shutdown()
        self.assertTrue(all(r[t] is not None for r in results for t in TOPICS), 'no output of some images')
        smoothed = [woman(r['dnn_pose_estimation/skeletons']) for r in results]
        raw = [woman(r['dnn_pose_estimation_raw/skeletons']) for r in results]
        before, during, after = smoothed[:split], smoothed[split:split + BLACK_IMAGES], smoothed[split + BLACK_IMAGES:]
        print('ids of the woman: %s' % ' '.join(str(s.id) if s else '-' for s in smoothed))

        # she is found in the most of the images (the person detection, which finds the persons by their faces,
        # misses her looking down at her phone), with the same id
        self.assertGreater(sum(s is not None for s in before), 0.8 * len(before))
        self.assertEqual(len({s.id for s in before if s}), 1, 'ids: %s' % [s.id if s else None for s in before])
        # nobody on the black images, and she is found again soon after them with the same id as before (the lost
        # persons keep their ids for a while)
        self.assertTrue(all(r['dnn_pose_estimation/skeletons'].skeletons == [] for r in
                            results[split:split + BLACK_IMAGES]))
        self.assertTrue(all(s is None for s in during))
        found_again = [s for s in after[:10] if s]
        self.assertTrue(found_again, 'not found again after the black images')
        self.assertEqual(found_again[0].id, before[-1].id if before[-1] else None)
        # after that, she is found in the most of the images, and her poses estimated while she looks down may be
        # lost and found again with a few new ids, then she keeps the same id
        self.assertGreater(sum(s is not None for s in after), 0.75 * len(after))
        self.assertLessEqual(len({s.id for s in after if s}), 3)
        self.assertEqual(len({s.id for s in after[-50:] if s}), 1)
        # the smoothing reduces the jitter of her visible landmarks
        smoothed_jitter, raw_jitter = median_acceleration(smoothed), median_acceleration(raw)
        print('median acceleration: smoothed %.2f, raw %.2f pixels' % (smoothed_jitter, raw_jitter))
        self.assertLess(smoothed_jitter, 0.8 * raw_jitter)
