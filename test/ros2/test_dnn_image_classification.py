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


# test of dnn_image_classification

import unittest

from opencv_apps.msg import DetectionArrayStamped
from opencv_apps_test import camera, description, image_view, include, MessagesTest, VSLAM_VIDEO
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(VSLAM_VIDEO),
        include('dnn_image_classification', image='image'),
        image_view('dnn_image_classification/image', name='dnn_image_classification_view'),
    )


def has_television(msg):
    # the 5 classes in the order of the score, and the television in the office of the video, by PP-ResNet50
    scores = [d.score for d in msg.detections]
    return len(msg.detections) == 5 and scores == sorted(scores, reverse=True) and sum(scores) <= 1.0 + 1e-3 and \
        any(d.label in ['television', 'home theater', 'entertainment center'] for d in msg.detections)


class TestDNNImageClassification(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_image_classification/detections', 'dnn_image_classification/image']
    CHECKS = {'dnn_image_classification/detections': (DetectionArrayStamped, has_television)}
