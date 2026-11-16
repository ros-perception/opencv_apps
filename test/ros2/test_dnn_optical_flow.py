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


# test of dnn_optical_flow (RAFT, OpenCV 4.10 or later), with an image of the vslam video and the image shifted by
# known pixels published one by one, as RAFT on the CPU takes about 1 s per image

import unittest

import cv2
import numpy as np
from opencv_apps.msg import FlowArrayStamped
from opencv_apps_test import description, image_view, include, publish_frames, video_frames, VSLAM_VIDEO
import pytest
import rclpy
from sensor_msgs.msg import Image

TOPICS = {'dnn_optical_flow/flows': FlowArrayStamped, 'dnn_optical_flow/image': Image}
# the shifts (pixels) of the second image from the first one, and of the third image from the second one
SHIFTS = [(8, 0), (0, 6)]


@pytest.mark.launch_test
def generate_test_description():
    return description(
        include('dnn_optical_flow', image='image'),
        image_view('dnn_optical_flow/image', name='dnn_optical_flow_view'),
    )


def shift(frame, dx, dy):
    return cv2.warpAffine(frame, np.float32([[1, 0, dx], [0, 1, dy]]), (frame.shape[1], frame.shape[0]),
                          borderMode=cv2.BORDER_REPLICATE)


class TestDNNOpticalFlow(unittest.TestCase):

    def test_flows(self):
        frames = video_frames(VSLAM_VIDEO, 0, 0)
        for dx, dy in SHIFTS:
            frames.append(shift(frames[-1], dx, dy))
        height, width = frames[0].shape[:2]
        rclpy.init()
        try:
            node = rclpy.create_node('test_dnn_optical_flow')
            # the first image waits for the model, downloaded at the first time
            results = publish_frames(node, frames, TOPICS, timeout=150.0)
        finally:
            rclpy.shutdown()
        self.assertTrue(all(r[t] is not None for r in results for t in TOPICS), 'no output of some images')
        # no flows of the first image
        self.assertEqual(results[0]['dnn_optical_flow/flows'].flow, [])
        for result, (dx, dy) in zip(results[1:], SHIFTS):
            image = result['dnn_optical_flow/image']
            self.assertEqual((image.width, image.height), (width, height))
            flows = result['dnn_optical_flow/flows'].flow
            # on the grid of 16 pixels, as fback_flow
            self.assertEqual(len(flows), ((width + 15) // 16) * ((height + 15) // 16))
            # the median of the flows inside the image (not at the borders replicated by the shift) is the shift
            velocities = np.array([(f.velocity.x, f.velocity.y) for f in flows
                                   if 32 <= f.point.x < width - 32 and 32 <= f.point.y < height - 32])
            median = np.median(velocities, axis=0)
            print('shift (%d, %d): median flow (%.2f, %.2f)' % (dx, dy, median[0], median[1]))
            self.assertLess(abs(median[0] - dx), 1.5)
            self.assertLess(abs(median[1] - dy), 1.5)
