#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Kei Okada.
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

# Set the target of the tracking node: call ~set_roi service with the ROI (and the image on which the ROI is given)

from __future__ import print_function

import argparse
import os
import sys
import time

if os.environ.get('ROS_VERSION') != '2':
    # rclpy on rospy, for ROS1
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'ros1', 'python'))
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from opencv_apps.msg import Rect
from opencv_apps.srv import SetROI


def main():
    parser = argparse.ArgumentParser(description='Send ROI (and the image on which the ROI is given) to the tracker')
    parser.add_argument('center_x', type=float, help='X coordinates of center of ROI')
    parser.add_argument('center_y', type=float, help='Y coordinates of center of ROI')
    parser.add_argument('width', type=float, help='Width of ROI')
    parser.add_argument('height', type=float, help='Height of ROI')
    parser.add_argument('image_file_name', nargs='?',
                        help='File name of the image on which the ROI is given (default: the next input image)')
    parser.add_argument('--debug-view', action='store_true', help='Display image_file_name with ROI')
    rclpy.init(args=sys.argv)
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    node = Node('set_image')
    logger = node.get_logger()
    req = SetROI.Request()

    # set roi
    logger.info("Set ROI({}, {}, {}, {})".format(args.center_x, args.center_y, args.width, args.height))
    req.rect = Rect(x=args.center_x, y=args.center_y, width=args.width, height=args.height)

    # set image
    if args.image_file_name:
        fname = args.image_file_name
        im = cv2.imread(fname)
        if im is None:
            logger.error("Failed to read {}, exiting".format(fname))
            return 1
        req.image = CvBridge().cv2_to_imgmsg(im, "bgr8")
        logger.info("Set Image{} from {}".format(im.shape, fname))
        if args.debug_view:
            cv2.rectangle(im,
                          (int(args.center_x - args.width / 2), int(args.center_y - args.height / 2)),
                          (int(args.center_x + args.width / 2), int(args.center_y + args.height / 2)),
                          (0, 255, 0), 3)
            cv2.imshow('template image', im)
            cv2.waitKey(100)

    set_roi = node.create_client(SetROI, 'set_roi')
    while not set_roi.wait_for_service(timeout_sec=1.0):
        if not rclpy.ok():
            return 1
        logger.info("Waiting for 'set_roi' service")
    future = set_roi.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()
    if res is None:
        return 1
    if res.ok:
        logger.info("'set_roi' succeeded")
    else:
        logger.error("'set_roi' failed: {}".format(res.error))
    if args.debug_view:
        time.sleep(2)
        cv2.destroyAllWindows()
    return 0 if res.ok else 1


if __name__ == '__main__':
    sys.exit(main())
