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


# ROS2 version of test/test-compare_histogram.test: the input is a BGR video (no image_proc for the gray image),
# so the reference histogram of the color node is computed from the first frame of the video.

import unittest

import cv2
from opencv_apps_test import camera, description, FACE_VIDEO, image_view, include, TopicsTest
import pytest


def reference_histogram(video, bins):
    """Flat [channel][bin] histogram of the first frame, normalized as ~/histogram of compare_histogram."""
    capture = cv2.VideoCapture(video)
    ok, img = capture.read()
    capture.release()
    assert ok, video
    values = []
    for c in range(img.shape[2]):
        hist = cv2.calcHist([img], [c], None, [bins], [0, 256])
        values += cv2.normalize(hist, None, 0, 1, cv2.NORM_MINMAX).flatten().tolist()
    return '[' + ', '.join('%.6f' % v for v in values) + ']'


@pytest.mark.launch_test
def generate_test_description():
    return description(
        camera(FACE_VIDEO),
        image_view('compare_histogram_color/image', name='image_view'),
        include('compare_histogram', image='image', histogram_size='64', node_name='compare_histogram_hist'),
        include('compare_histogram', image='image', histogram_size='32', histogram_comparison_type='3',
                reference_histogram=reference_histogram(FACE_VIDEO, 32), node_name='compare_histogram_color'),
        # integers and decimals over multiple lines, as the ROS1 test writes it
        include('compare_histogram', image='image', histogram_size='4', histogram_comparison_type='1',
                reference_histogram='''
                  [0, 0.5, 1, 0,
                   0, 0.5, 1, 0,
                   1, 0.5, 0, 1]''', node_name='compare_histogram_int'),
    )


class TestCompareHistogram(TopicsTest, unittest.TestCase):

    TOPICS = ['compare_histogram_hist/image', 'compare_histogram_hist/histogram', 'compare_histogram_color/image',
              'compare_histogram_color/histogram', 'compare_histogram_color/distance',
              'compare_histogram_color/channel_distances', 'compare_histogram_int/distance']
