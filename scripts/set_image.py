#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, Kei Okada
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


from __future__ import print_function

try:
    input = raw_input
except:
    pass

import argparse
import cv2
import cv_bridge
import os
import sys
import time
import rospy

from opencv_apps.msg import Rect
from opencv_apps.srv import SetImages, SetImagesRequest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send ROI (and template image) to the tracker')
    parser.add_argument('center_x', type=int, help='X coordinates of center of ROI')
    parser.add_argument('center_y', type=int, help='Y coordinates of center of ROI')
    parser.add_argument('width', type=int, help='Width of ROI')
    parser.add_argument('height', type=int, help='Height of ROI')
    parser.add_argument('image_file_name', nargs='?', help='File name of template image')
    parser.add_argument('--debug-view', action='store_true', help='Display input_file with ROI')
    args, unknown = parser.parse_known_args();

    rospy.init_node("set_image")
    rospy.wait_for_service('set_roi')
    set_roi = rospy.ServiceProxy('set_roi', SetImages)

    req = SetImagesRequest()

    # set roi
    rospy.loginfo("Set ROI({},{}, {}, {})".format(args.center_x, args.center_y, args.width, args.height))
    rect = Rect(x=args.center_x, y=args.center_y, width=args.width, height=args.height)
    req.rects=[rect]

    # set images
    if args.image_file_name:
        fname = args.image_file_name
        if os.path.exists(fname):
            im = cv2.imread(fname)
            im_msg = cv_bridge.CvBridge().cv2_to_imgmsg(im, "bgr8")
            rospy.loginfo("Set Image{} from {}".format(im.shape, fname))
            req.images = [im_msg]
            if args.debug_view:
                cv2.rectangle(im,
                              (int(rect.x - rect.width/2), int(rect.y - rect.height/2)),
                              (int(rect.x + rect.width/2), int(rect.y + rect.height/2)),
                              (0, 255, 0), 3)
                cv2.imshow('template image', im)
                cv2.waitKey(100)
        else:
            rospy.logerr("{} not found exitting".format(fname))
            sys.exit(1)

    ret = set_roi(images=req.images, rects=req.rects)
    rospy.loginfo("'set_roi' returns\n {}".format(ret))
    if args.debug_view:
        time.sleep(2)
        cv2.destroyAllWindows()
