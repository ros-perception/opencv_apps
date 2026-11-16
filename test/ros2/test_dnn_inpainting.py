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


# test of dnn_inpainting (OpenCV 5.0 or later), with the frames of the face video published one by one: the image is
# published as it is without the region to fill, the rect of ~set_roi (the face of the person sitting at the center)
# and then the mask on mask are filled, and the pixels outside of them are not changed

import time
import unittest

import numpy as np
from opencv_apps.msg import Rect
from opencv_apps.srv import SetROI
from opencv_apps_test import description, FACE_VIDEO, image_view, include, publish_frames, video_frames
import pytest
import rclpy
from sensor_msgs.msg import Image

TOPICS = {'dnn_inpainting/image': Image}
# the face of the person sitting at the center (center x, y, width, height), as test_tracking.py
FACE = (377.0, 211.0, 50.0, 50.0)
# the region of the mask (x, y, width, height), on the left of the face
MASK = (200, 250, 60, 80)


@pytest.mark.launch_test
def generate_test_description():
    return description(
        include('dnn_inpainting', image='image'),
        image_view('dnn_inpainting/image', name='dnn_inpainting_view'),
    )


def to_image(msg):
    return np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)


class TestDNNInpainting(unittest.TestCase):

    TIMEOUT = 120.0

    def set_roi(self, node, x, y, width, height):
        client = node.create_client(SetROI, 'dnn_inpainting/set_roi')
        self.assertTrue(client.wait_for_service(timeout_sec=self.TIMEOUT))
        future = client.call_async(SetROI.Request(rect=Rect(x=x, y=y, width=width, height=height)))
        rclpy.spin_until_future_complete(node, future, timeout_sec=self.TIMEOUT)
        self.assertTrue(future.result() and future.result().ok)

    def check_filled(self, frame, msg, x, y, width, height):
        """Check that msg is the frame, whose region (top-left x, y) is changed and the others are not."""
        self.assertIsNotNone(msg, 'no output')
        self.assertEqual((msg.height, msg.width, msg.encoding), (frame.shape[0], frame.shape[1], 'bgr8'))
        output = to_image(msg)
        inside = np.zeros(frame.shape[:2], bool)
        inside[y:y + height, x:x + width] = True
        difference = np.abs(output.astype(int) - frame.astype(int))
        print('mean difference in the region: %.1f' % difference[inside].mean())
        self.assertGreater(difference[inside].mean(), 5.0)
        self.assertEqual(np.count_nonzero(difference[~inside]), 0)

    def test_inpainting(self):
        frames = video_frames(FACE_VIDEO, 0, 2)
        rclpy.init()
        try:
            node = rclpy.create_node('test_dnn_inpainting')
            # no region to fill, the image as it is
            result = publish_frames(node, frames[:1], TOPICS)[0]['dnn_inpainting/image']
            self.assertIsNotNone(result, 'no output')
            self.assertTrue(np.array_equal(to_image(result), frames[0]))

            # the rect of ~set_roi, the first image waits for the model, downloaded at the first time
            self.set_roi(node, *FACE)
            start = time.time()
            result = publish_frames(node, frames[1:2], TOPICS, timeout=self.TIMEOUT)[0]['dnn_inpainting/image']
            print('the first inpainting (with loading the model) took %.2f s' % (time.time() - start))
            x, y, width, height = FACE
            self.check_filled(frames[1], result, int(x - width / 2), int(y - height / 2), int(width), int(height))

            # the mask, after the rect is cleared by an empty rect
            self.set_roi(node, 0.0, 0.0, 0.0, 0.0)
            mask = np.zeros(frames[2].shape[:2], np.uint8)
            x, y, width, height = MASK
            mask[y:y + height, x:x + width] = 255
            publisher = node.create_publisher(Image, 'mask', 10)
            end_time = time.time() + self.TIMEOUT
            while publisher.get_subscription_count() == 0 and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
            publisher.publish(Image(height=mask.shape[0], width=mask.shape[1], encoding='mono8', step=mask.shape[1],
                                    data=mask.tobytes()))
            # wait for the mask to be received
            end_time = time.time() + 1.0
            while time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
            start = time.time()
            result = publish_frames(node, frames[2:3], TOPICS, timeout=self.TIMEOUT)[0]['dnn_inpainting/image']
            print('inpainting took %.2f s' % (time.time() - start))
            self.check_filled(frames[2], result, x, y, width, height)
        finally:
            rclpy.shutdown()
