#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
#  * Neither the name of the JSK Lab nor the names of its
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

# Make test/text.mp4, the input of the tests of dnn_text_detection: words printed on a sheet, which moves in the
# view as test/codes.mp4 of make_codes_video.py.
#   python3 test/make_text_video.py test/text.mp4

import sys

import cv2
from make_codes_video import write_video
import numpy as np

WORDS = [('OPENCV', 3.0), ('APPS', 3.0), ('ROS 2026', 2.0)]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'text.mp4'
    sheet = np.full((600, 800, 3), 255, np.uint8)
    y = 40
    for text, scale in WORDS:
        (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, scale, 6)
        y += height + 70
        cv2.putText(sheet, text, ((800 - width) // 2, y), cv2.FONT_HERSHEY_DUPLEX, scale, (40, 40, 40), 6,
                    cv2.LINE_AA)
    write_video(sheet, out)


if __name__ == '__main__':
    main()
