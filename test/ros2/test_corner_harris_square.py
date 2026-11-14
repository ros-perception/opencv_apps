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


# corner_harris on an image of a white square: ~/corners has one point at each of the 4 corners

import os
import tempfile
import unittest

import cv2
import numpy as np
from opencv_apps.msg import Point2DArrayStamped
from opencv_apps_test import camera, description, include, MessagesTest
import pytest

SQUARE = os.path.join(tempfile.mkdtemp(prefix='opencv_apps_test_'), 'square.png')
CORNERS = [(80, 60), (240, 60), (80, 180), (240, 180)]


def write_square():
    image = np.zeros((240, 320, 3), np.uint8)
    cv2.rectangle(image, CORNERS[0], CORNERS[3], (255, 255, 255), -1)
    cv2.imwrite(SQUARE, image)


@pytest.mark.launch_test
def generate_test_description():
    write_square()
    return description(camera(SQUARE), include('corner_harris', image='image', threshold='200'))


def four_corners(msg):
    # one point near each corner of the square, not a point for each pixel of a corner
    return len(msg.points) == 4 and all(any(abs(p.x - x) <= 3 and abs(p.y - y) <= 3 for p in msg.points)
                                        for x, y in CORNERS)


class TestCornerHarrisSquare(MessagesTest, unittest.TestCase):

    TOPICS = ['corner_harris/corners']
    CHECKS = {'corner_harris/corners': (Point2DArrayStamped, four_corners)}
