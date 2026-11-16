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


# test of dnn_edge_detection

import unittest

import numpy as np
from opencv_apps_test import camera, description, image_view, include, MessagesTest, VSLAM_VIDEO
import pytest
from sensor_msgs.msg import Image


def edges(msg):
    """The edges in mono8 of the size of the video (640x480), with some edge pixels and not all."""
    if msg.encoding != 'mono8' or (msg.width, msg.height) != (640, 480):
        return False
    image = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
    # 13 % of the pixels by DexiNed
    return 0.02 < np.count_nonzero(image > 128) / image.size < 0.5


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(VSLAM_VIDEO),
        include('dnn_edge_detection', image='image'),
        image_view('dnn_edge_detection/image', name='dnn_edge_detection_view'),
    )


class TestDNNEdgeDetection(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_edge_detection/image']
    CHECKS = {'dnn_edge_detection/image': (Image, edges)}
