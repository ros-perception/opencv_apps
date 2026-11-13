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


# ROS2 test of tracking with the person walking from the right to the left of FACE_VIDEO: the face on the first
# frame is set by ~set_roi, then the frames are published one by one, each after the output of the previous one, so
# that the result does not depend on the speed of the tracker. CSRT tracker is in opencv_contrib's tracking module.

import time
import unittest

import cv2
from opencv_apps.msg import Rect, RectArrayStamped
from opencv_apps.srv import SetROI
from opencv_apps_test import description, FACE_VIDEO, image_view, include
import pytest
import rclpy
from sensor_msgs.msg import Image

# the face of the walking person at the first frame (center x, y, width, height), and the last frame of the walk
FACE = (560.0, 170.0, 50.0, 60.0)
LAST_FRAME = 96


def walk_description(name, **args):
    return description(
        image_view(name + '/image', name='image_view'),
        include(name, image='image', **args),
    )


@pytest.mark.launch_test
def generate_test_description():
    return walk_description('tracking', tracking_algorithm=7)


class TestTrackingWalk(unittest.TestCase):

    NODE = 'tracking'
    TIMEOUT = 120.0

    def test_walk(self):
        cap = cv2.VideoCapture(FACE_VIDEO)
        frames = [cap.read()[1] for _ in range(LAST_FRAME + 1)]
        rclpy.init()
        try:
            node = rclpy.create_node('test_' + self.__class__.__name__.lower())
            outputs = {}
            node.create_subscription(RectArrayStamped, self.NODE + '/output',
                                     lambda msg: outputs.__setitem__(msg.header.stamp.sec, msg.rects), 10)
            pub = node.create_publisher(Image, 'image', 10)

            def image(i):
                f = frames[i]
                msg = Image(height=f.shape[0], width=f.shape[1], encoding='bgr8', step=f.shape[1] * 3,
                            data=f.tobytes())
                msg.header.frame_id = 'camera'
                msg.header.stamp.sec = i  # the frame number, to find its output
                return msg

            end_time = time.time() + self.TIMEOUT
            client = node.create_client(SetROI, self.NODE + '/set_roi')
            self.assertTrue(client.wait_for_service(timeout_sec=self.TIMEOUT))
            x, y, w, h = FACE
            # the DNN trackers may download their models here
            future = client.call_async(SetROI.Request(image=image(0), rect=Rect(x=x, y=y, width=w, height=h)))
            rclpy.spin_until_future_complete(node, future, timeout_sec=self.TIMEOUT)
            self.assertTrue(future.result() and future.result().ok)
            # wait for the subscriber of the tracker
            while pub.get_subscription_count() == 0 and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
            for i in range(1, LAST_FRAME + 1):
                frame_end_time = min(end_time, time.time() + 10.0)
                pub.publish(image(i))
                while i not in outputs and time.time() < frame_end_time:
                    rclpy.spin_once(node, timeout_sec=0.1)
                self.assertIn(i, outputs, 'no output of frame %d' % i)
                self.assertEqual(len(outputs[i]), 1, 'lost the face at frame %d' % i)
                # the face moves a little between the frames
                previous = outputs[i - 1][0] if i > 1 else Rect(x=x, y=y)
                self.assertLess(abs(outputs[i][0].x - previous.x), 60, 'jumped at frame %d' % i)
                self.assertLess(abs(outputs[i][0].y - previous.y), 60, 'jumped at frame %d' % i)
            print('x of the face: %s' % [round(outputs[i][0].x) for i in range(1, LAST_FRAME + 1)])
            # the face moved from the right (x = 560) to the left (x = 130 or so)
            self.assertLess(outputs[LAST_FRAME][0].x, 200)
        finally:
            rclpy.shutdown()
