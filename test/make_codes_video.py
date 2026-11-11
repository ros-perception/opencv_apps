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

# Make test/codes.mp4, the input of the tests of qrcode_detection, barcode_detection and aruco_detection:
# a QR code, an EAN-13 barcode and two ArUco markers (DICT_4X4_50) on a sheet, which moves in the view.
#   python3 test/make_codes_video.py test/codes.mp4

import os
import subprocess
import sys

import cv2
import numpy as np

QR_TEXT = 'opencv_apps'
EAN13 = '4901234567894'
ARUCO_IDS = [1, 7]


def ean13(code, module=3, height=120):
    L = ['0001101', '0011001', '0010011', '0111101', '0100011', '0110001', '0101111', '0111011', '0110111',
         '0001011']
    G = ['0100111', '0110011', '0011011', '0100001', '0011101', '0111001', '0000101', '0010001', '0001001',
         '0010111']
    R = ['1110010', '1100110', '1101100', '1000010', '1011100', '1001110', '1010000', '1000100', '1001000',
         '1110100']
    PARITY = ['LLLLLL', 'LLGLGG', 'LLGGLG', 'LLGGGL', 'LGLLGG', 'LGGLLG', 'LGGGLL', 'LGLGLG', 'LGLGGL', 'LGGLGL']
    d = [int(c) for c in code]
    bits = '101'
    for i in range(6):
        bits += (L if PARITY[d[0]][i] == 'L' else G)[d[i + 1]]
    bits += '01010'
    for i in range(6):
        bits += R[d[i + 7]]
    bits += '101'
    quiet = 10
    img = np.full((height, (len(bits) + 2 * quiet) * module), 255, np.uint8)
    for i, b in enumerate(bits):
        if b == '1':
            img[:, (quiet + i) * module:(quiet + i + 1) * module] = 0
    return img


def aruco(marker_id, size):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):  # OpenCV >= 4.7
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, size)
    return cv2.copyMakeBorder(marker, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)


def paste(sheet, img, x, y):
    sheet[y:y + img.shape[0], x:x + img.shape[1]] = img


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'codes.mp4'
    sheet = np.full((600, 800), 255, np.uint8)
    qr = cv2.QRCodeEncoder.create().encode(QR_TEXT)
    qr = cv2.resize(qr, (qr.shape[1] * 11, qr.shape[0] * 11), interpolation=cv2.INTER_NEAREST)
    paste(sheet, cv2.copyMakeBorder(qr, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255), 20, 20)
    paste(sheet, ean13(EAN13), 400, 60)
    paste(sheet, aruco(ARUCO_IDS[0], 160), 400, 300)
    paste(sheet, aruco(ARUCO_IDS[1], 160), 600, 300)
    sheet = cv2.cvtColor(sheet, cv2.COLOR_GRAY2BGR)

    w, h, fps, frames = 640, 480, 10, 100
    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    sh, sw = sheet.shape[:2]
    src = np.float32([[0, 0], [sw, 0], [sw, sh], [0, sh]])
    for i in range(frames):
        t = 2 * np.pi * i / frames
        # the sheet moves around and tilts a little in the view
        cx, cy, scale, angle = w / 2 + 20 * np.sin(t), h / 2 + 15 * np.sin(2 * t), 0.66 + 0.04 * np.sin(t), \
            0.15 * np.sin(t)
        corners = np.array([[-sw / 2, -sh / 2], [sw / 2, -sh / 2], [sw / 2, sh / 2], [-sw / 2, sh / 2]]) * scale
        rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        corners = corners.dot(rot.T)
        corners[:, 0] *= 1 + 0.08 * np.sin(t) * np.sign(corners[:, 1])  # perspective
        dst = np.float32(corners + [cx, cy])
        frame = cv2.warpPerspective(sheet, cv2.getPerspectiveTransform(src, dst), (w, h),
                                    borderMode=cv2.BORDER_CONSTANT, borderValue=(90, 110, 130))
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        writer.write(frame)
    writer.release()
    # smaller file with x264, if ffmpeg is installed
    try:
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', out, '-c:v', 'libx264', '-crf', '23',
                        '-pix_fmt', 'yuv420p', out + '.mp4'], check=True)
        os.rename(out + '.mp4', out)
    except (OSError, subprocess.CalledProcessError):
        pass


if __name__ == '__main__':
    main()
