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


# ROS2 test of dnn_interactive_segmentation: the box around the woman standing at the center is given by set_image.py
# as the tracking nodes, and she is segmented with EfficientSAM (OpenCV 4.10 or later), downloaded at the first time.

import unittest

import launch_ros.actions
import numpy as np
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, MessagesTest
import pytest
from sensor_msgs.msg import Image

# the box of set_roi: x-y is the center, as opencv_apps/Rect
BOX = (375, 325, 150, 310)


def inside_box(msg):
    """Return True if the mask is not small, and mostly inside the box (with a margin)."""
    if msg.encoding != 'mono8':
        return False
    mask = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width] > 0
    x, y, width, height = BOX
    margin = 20
    inside = mask[max(0, int(y - height / 2 - margin)):int(y + height / 2 + margin),
                  max(0, int(x - width / 2 - margin)):int(x + width / 2 + margin)].sum()
    # the woman covers about half of the box
    return inside > 0.2 * width * height and inside > 0.9 * mask.sum()


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        include('dnn_interactive_segmentation', image='image'),
        image_view('dnn_interactive_segmentation/image', name='dnn_interactive_segmentation_view'),
        launch_ros.actions.Node(package='opencv_apps', executable='set_image.py',
                                name='dnn_interactive_segmentation_set_roi', arguments=[str(v) for v in BOX],
                                remappings=[('set_roi', 'dnn_interactive_segmentation/set_roi')]),
    )


class TestDNNInteractiveSegmentation(MessagesTest, unittest.TestCase):

    TOPICS = ['dnn_interactive_segmentation/image', 'dnn_interactive_segmentation/mask',
              'dnn_interactive_segmentation/contours', 'dnn_interactive_segmentation/output']
    CHECKS = {'dnn_interactive_segmentation/mask': (Image, inside_box)}
