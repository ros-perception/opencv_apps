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
# test of dnn_person_reid with the images published continuously at 30 Hz and their detections 0.6 s later (the
# latency of a detector): the images wait for their detections in the synchronizer (sync_queue_size), and the
# detections are published by dnn_person_reid

import os
import time
import unittest

from opencv_apps.msg import Detection, DetectionArrayStamped, Rect
from opencv_apps_test import description, include, TEST_DIR, video_frames
import pytest
import rclpy
from sensor_msgs.msg import Image

RATE = 30.0
DELAY = 0.6
DURATION = 3.0


@pytest.mark.launch_test
def generate_test_description():
    return description(include('dnn_person_reid', image='image', input='detections'))


class TestDNNPersonReIDDelay(unittest.TestCase):

    def test_delayed_detections(self):
        frames = video_frames(os.path.join(TEST_DIR, 'rgbd_walking.mp4'), 0, 19)
        rclpy.init()
        try:
            node = rclpy.create_node('test_dnn_person_reid_delay')
            received = []
            node.create_subscription(DetectionArrayStamped, 'dnn_person_reid/detections',
                                     lambda msg: received.append(msg), 10)
            image_pub = node.create_publisher(Image, 'image', 10)
            detections_pub = node.create_publisher(DetectionArrayStamped, 'detections', 10)
            # dnn_person_reid subscribes the image and the detections once ~/detections is subscribed
            end_time = time.time() + 30.0
            while image_pub.get_subscription_count() == 0 and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertGreater(image_pub.get_subscription_count(), 0)

            count = [0]

            def publish(delay, until):
                """Publish the images at RATE until the time, each detection delay seconds after its image."""
                pending = []
                next_time = time.time()
                while time.time() < until or pending:
                    now = time.time()
                    if now >= next_time and now < until:
                        frame = frames[count[0] % len(frames)]
                        image = Image(height=frame.shape[0], width=frame.shape[1], encoding='bgr8',
                                      step=frame.shape[1] * 3, data=frame.tobytes())
                        image.header.frame_id = 'camera'
                        image.header.stamp = node.get_clock().now().to_msg()
                        image_pub.publish(image)
                        # a person at the center of the image
                        detections = DetectionArrayStamped(header=image.header, detections=[Detection(
                            label='person', id=0, score=0.9, rect=Rect(x=320.0, y=240.0, width=160.0, height=400.0))])
                        pending.append((now + delay, detections))
                        count[0] += 1
                        next_time += 1.0 / RATE
                    while pending and pending[0][0] <= now:
                        detections_pub.publish(pending.pop(0)[1])
                    rclpy.spin_once(node, timeout_sec=0.002)

            # the model is downloaded at the first time: wait for the first output, without the delay
            end_time = time.time() + 120.0
            while not received and time.time() < end_time:
                publish(0.0, time.time() + 0.5)
            self.assertTrue(received, 'no output of dnn_person_reid')
            received.clear()
            start = count[0]
            publish(DELAY, time.time() + DURATION)
            end_time = time.time() + 2.0
            while time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            rclpy.shutdown()
        published = count[0] - start
        print('%d of %d images with the delayed detections are published' % (len(received), published))
        # most of them, not the few of the images still in a small queue (e.g. 10 of 18 images within 0.6 s)
        self.assertGreater(len(received), 0.8 * published)
        self.assertTrue(all(len(msg.detections) == 1 for msg in received))
