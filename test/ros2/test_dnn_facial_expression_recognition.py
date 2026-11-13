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


# test of dnn_facial_expression_recognition

import unittest

from opencv_apps.msg import FaceArrayStamped
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, MessagesTest
import pytest

EXPRESSIONS = ['angry', 'disgust', 'fearful', 'happy', 'neutral', 'sad', 'surprised']


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        include('dnn_facial_expression_recognition', image='image'),
        image_view('dnn_facial_expression_recognition/image', name='dnn_facial_expression_recognition_view'),
    )


class TestDNNFacialExpressionRecognition(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_facial_expression_recognition/faces', 'dnn_facial_expression_recognition/image']
    # a face of the persons in the video with its expression
    CHECKS = {'dnn_facial_expression_recognition/faces': (FaceArrayStamped, lambda msg: any(
        f.label in EXPRESSIONS and 1.0 / len(EXPRESSIONS) < f.confidence <= 1.0 for f in msg.faces))}
