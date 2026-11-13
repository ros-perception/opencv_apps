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


# ROS2 version of test/test-match_template.test: the template (the face at the center) is cropped from the first
# frame of the video, and given by ~template_file and ~mask_file, published on ~template, or set by ~set_roi with
# set_image.py as the tracking node.

import atexit
import os
import shutil
import tempfile
import unittest

import cv2
import launch_ros.actions
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import numpy as np
import pytest
from sensor_msgs.msg import Image

FACE_RECT = (348, 176, 60, 64)  # x, y, width, height of the face in the first frame


def face_template():
    ok, img = cv2.VideoCapture(FACE_VIDEO).read()
    assert ok, FACE_VIDEO
    x, y, w, h = FACE_RECT
    return img[y:y + h, x:x + w].copy()


def template_files():
    path = tempfile.mkdtemp(prefix='opencv_apps_test_')
    atexit.register(shutil.rmtree, path, ignore_errors=True)
    templ = face_template()
    mask = np.zeros(templ.shape[:2], np.uint8)
    cv2.ellipse(mask, (templ.shape[1] // 2, templ.shape[0] // 2), (templ.shape[1] // 2, templ.shape[0] // 2), 0, 0,
                360, 255, -1)
    cv2.imwrite(os.path.join(path, 'template.png'), templ)
    cv2.imwrite(os.path.join(path, 'mask.png'), mask)
    return os.path.join(path, 'template.png'), os.path.join(path, 'mask.png')


@pytest.mark.launch_test
def generate_test_description():
    template_file, mask_file = template_files()
    return description(
        camera(FACE_VIDEO),
        image_view('match_template/image', name='image_view'),
        include('match_template', image='image', template_file=template_file),
        include('match_template', image='image', template_file=template_file, mask_file=mask_file, match_method='3',
                match_threshold='0.9', max_matches='5', node_name='match_template_mask'),
        include('match_template', image='image', match_method='1', node_name='match_template_topic'),
        include('match_template', image='image', node_name='match_template_roi'),
        # the face of the next input image is the template of match_template_roi
        launch_ros.actions.Node(package='opencv_apps', executable='set_image.py', name='match_template_set_roi',
                                arguments=['378', '208', '60', '64'],
                                remappings=[('set_roi', 'match_template_roi/set_roi')]),
    )


class TestMatchTemplate(TopicsTest, unittest.TestCase):

    TOPICS = ['match_template/image', 'match_template/matched_image', 'match_template/output', 'match_template/score',
              'match_template_mask/image', 'match_template_mask/output',
              'match_template_topic/image', 'match_template_topic/output',
              'match_template_roi/image', 'match_template_roi/output']

    def prepare(self, node):
        # match_template_topic has no template_file, publish the template on ~/template until the test ends
        templ = face_template()
        msg = Image(height=templ.shape[0], width=templ.shape[1], encoding='bgr8', step=templ.shape[1] * 3,
                    data=templ.tobytes())
        self.template_pub = node.create_publisher(Image, '/match_template_topic/template', 1)
        self.timer = node.create_timer(0.5, lambda: self.template_pub.publish(msg))

