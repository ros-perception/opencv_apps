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



# test of dnn_super_resolution: ESPCN x2 (default) and FSRCNN x3 of dnn_superres (OpenCV 4.10 or later), whose
# ~/image is 2 and 3 times as large as the 640x480 images of FACE_VIDEO, with the colors of the images

import unittest

import cv2
import numpy as np
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, MessagesTest
import pytest
from sensor_msgs.msg import Image

FSRCNN_X3_MODEL = ('https://raw.githubusercontent.com/opencv/opencv_extra/697e5c3de828f3154e82f222dbe902d622bbe19a/'
                   'testdata/cv/dnn_superres/FSRCNN_x3.onnx')


def video_means(video):
    """The minimum and the maximum of the mean of the B, G and R channels of each image of the video."""
    capture = cv2.VideoCapture(video)
    means = []
    while True:
        ok, image = capture.read()
        if not ok:
            break
        means.append(image.reshape(-1, 3).mean(axis=0))
    return np.min(means, axis=0), np.max(means, axis=0)


MEANS = video_means(FACE_VIDEO)


def enlarged(scale):
    def check(msg):
        """The image is scale times as large as the input, and its colors are those of the input."""
        if msg.encoding != 'bgr8' or (msg.width, msg.height) != (640 * scale, 480 * scale):
            return False
        mean = np.frombuffer(bytes(msg.data), np.uint8).reshape(-1, 3).mean(axis=0)
        return bool(np.all(MEANS[0] - 3 < mean) and np.all(mean < MEANS[1] + 3))
    return check


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        include('dnn_super_resolution', image='image'),
        include('dnn_super_resolution', image='image', node_name='dnn_super_resolution_x3', model=FSRCNN_X3_MODEL),
        image_view('dnn_super_resolution/image', name='dnn_super_resolution_view'),
    )


class TestDNNSuperResolution(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_super_resolution/image', 'dnn_super_resolution_x3/image']
    CHECKS = {'dnn_super_resolution/image': (Image, enlarged(2)),
              'dnn_super_resolution_x3/image': (Image, enlarged(3))}
