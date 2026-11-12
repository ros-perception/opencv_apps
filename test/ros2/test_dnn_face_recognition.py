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


# test of dnn_face_recognition

import tempfile
import unittest

from opencv_apps.msg import FaceArrayStamped
from opencv_apps.srv import FaceRecognitionTrain
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, MessagesTest
import pytest
import rclpy
from sensor_msgs.msg import Image


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        include('dnn_face_recognition', image='image', data_dir=tempfile.mkdtemp(prefix='opencv_apps_test_')),
        image_view('dnn_face_recognition/image', name='dnn_face_recognition_view'),
    )


class TestDNNFaceRecognition(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_face_recognition/output', 'dnn_face_recognition/image']
    # the trained face is recognized in the other frames
    CHECKS = {'dnn_face_recognition/output': (FaceArrayStamped, lambda msg: any(
        f.label == 'test_person' for f in msg.faces))}

    def prepare(self, node):
        # train the face in a frame of the camera, as face_recognition_trainer.py
        images = []
        subscription = node.create_subscription(Image, 'image', images.append, 10)
        client = node.create_client(FaceRecognitionTrain, 'dnn_face_recognition/train')
        self.assertTrue(client.wait_for_service(timeout_sec=120.0))
        # the face may not be detected in some frames, then try the later frames
        for _ in range(10):
            del images[:]
            while len(images) < 10:
                rclpy.spin_once(node, timeout_sec=0.1)
            future = client.call_async(FaceRecognitionTrain.Request(images=[images[-1]], labels=['test_person']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=120.0)
            if future.result().ok:
                break
        node.destroy_subscription(subscription)
        self.assertTrue(future.result().ok, future.result().error)
