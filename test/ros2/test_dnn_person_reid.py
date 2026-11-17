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

# test of dnn_person_reid with the people detected by dnn_object_detection in the frames of test/rgbd_walking.mp4
# (TUM RGB-D) published one by one: the two people, one on the left and one on the right, keep their ids over the
# images, and are re-identified with the same ids after they are lost on black images

import os
import unittest

import numpy as np
from opencv_apps.msg import DetectionArrayStamped
from opencv_apps_test import description, image_view, include, publish_frames, TEST_DIR, video_frames
import pytest
import rclpy

TOPIC = 'dnn_person_reid/detections'
# black images inserted after the frames, which are published again after them
BLACK_IMAGES = 3


@pytest.mark.launch_test
def generate_test_description():
    return description(
        include('dnn_object_detection', image='image'),
        include('dnn_person_reid', image='image', input='dnn_object_detection/detections'),
        image_view('dnn_person_reid/image'),
    )


def ids(msg, left):
    """Return the ids of the people on the left (the center x < 320) or on the right of the image."""
    return [int(d.data) for d in msg.detections if d.label == 'person' and d.score > 0.5 and (d.rect.x < 320) == left]


class TestDNNPersonReID(unittest.TestCase):

    def test_reid(self):
        frames = video_frames(os.path.join(TEST_DIR, 'rgbd_walking.mp4'), 0, 19)
        frames = frames + [np.zeros_like(frames[0])] * BLACK_IMAGES + frames
        rclpy.init()
        try:
            node = rclpy.create_node('test_dnn_person_reid')
            # the first image waits for the models, downloaded at the first time; dnn_object_detection and
            # dnn_person_reid subscribe the images
            results = publish_frames(node, frames, {TOPIC: DetectionArrayStamped}, timeout=120.0, subscribers=2)
        finally:
            rclpy.shutdown()
        self.assertTrue(all(r[TOPIC] is not None for r in results), 'no output of some images')
        msgs = [r[TOPIC] for r in results]
        for msg in msgs:
            print(' '.join('%s_%s(x=%.0f, width=%.0f)' % (d.label, d.data, d.rect.x, d.rect.width)
                           for d in msg.detections) or '-')
        # nobody on the black images
        self.assertTrue(all(msg.detections == [] for msg in msgs[20:20 + BLACK_IMAGES]))
        people = msgs[:20] + msgs[20 + BLACK_IMAGES:]
        left = [i for msg in people for i in ids(msg, True)]
        right = [i for msg in people for i in ids(msg, False)]
        # both of them are found in the most of the images, each with one id, also after the black images
        self.assertGreater(len(left), 0.8 * len(people))
        self.assertGreater(len(right), 0.8 * len(people))
        self.assertEqual(len(set(left)), 1, 'ids of the person on the left: %s' % left)
        self.assertEqual(len(set(right)), 1, 'ids of the person on the right: %s' % right)
        self.assertNotEqual(left[0], right[0])
        # the class id of dnn_object_detection is kept (person of COCO is 0), the id of the person is in the data
        self.assertTrue(all(d.id == 0 for msg in people for d in msg.detections if d.label == 'person'))
