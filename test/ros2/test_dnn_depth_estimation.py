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


# test of dnn_depth_estimation

import unittest

import numpy as np
from opencv_apps_test import camera, description, image_view, include, MessagesTest, VSLAM_VIDEO
import pytest
from sensor_msgs.msg import Image


def nearer_bottom(msg):
    """The floor at the bottom of the view is nearer (larger inverse depth) than the middle of the view."""
    if msg.encoding != '32FC1':
        return False
    inverse_depth = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width)
    bottom = np.median(inverse_depth[-msg.height // 8:])
    middle = np.median(inverse_depth[msg.height * 3 // 8:msg.height // 2])
    # 1.6 - 4.3 times by MiDaS v2.1 small
    return bottom > 1.2 * middle > 0


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(VSLAM_VIDEO),
        include('dnn_depth_estimation', image='image'),
        image_view('dnn_depth_estimation/image', name='dnn_depth_estimation_view'),
    )


class TestDNNDepthEstimation(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_depth_estimation/inverse_depth', 'dnn_depth_estimation/image']
    CHECKS = {'dnn_depth_estimation/inverse_depth': (Image, nearer_bottom)}
