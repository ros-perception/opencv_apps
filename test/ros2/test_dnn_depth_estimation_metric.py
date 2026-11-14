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


# the metric depth estimated by dnn_depth_estimation with Depth Anything V2 Metric (Hypersim, indoors) of
# https://huggingface.co/k-okada/depth-anything-v2-metric-small-onnx (OpenCV 4.10 or later) in the color images of
# test/rgbd_walking.mp4 (TUM RGB-D, about 1 - 4 m), published by test/publish_rgbd.py

import os
import sys
import unittest

import launch.actions
import numpy as np
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest
from sensor_msgs.msg import Image

MODEL = ('https://huggingface.co/k-okada/depth-anything-v2-metric-small-onnx/resolve/'
         '5494458701c7274fa82af484e2f2d226c948b54d/depth_anything_v2_metric_hypersim_vits_518.onnx')


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_rgbd.py')],
                                      output='screen'),
        include('dnn_depth_estimation', image='image', model=MODEL, input_width='518', input_height='518',
                metric_depth='true'),
    )


def metric(msg):
    # the depth in meters of the room, not the relative inverse depth of MiDaS
    depth = np.frombuffer(bytes(msg.data), np.float32)
    return msg.encoding == '32FC1' and 1.0 < float(np.median(depth)) < 5.0


class TestDNNDepthEstimationMetric(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_depth_estimation/depth', 'dnn_depth_estimation/image']
    CHECKS = {'dnn_depth_estimation/depth': (Image, metric)}
