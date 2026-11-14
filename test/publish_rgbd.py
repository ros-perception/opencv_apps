#!/usr/bin/env python3
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

# Publish an RGB-D sequence in a loop: image (bgr8), depth (16UC1 in mm, registered to the image) and camera_info,
# with the same stamps, from the color video and the depths in .npz (array "depth", uint16 mm), by default
# test/rgbd_walking.mp4 and test/rgbd_walking_depth.npz (20 images of the TUM RGB-D freiburg3_walking_xyz sequence,
# CC BY 4.0, https://cvg.cit.tum.de/data/datasets/rgbd-dataset, every 6th of its first 120 images).
#   publish_rgbd.py [VIDEO DEPTH_NPZ] [--rate 10]

import argparse
import os
import sys

import cv2
import numpy as np

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
# the intrinsics of the freiburg3 sequences, whose images are undistorted
FX, FY, CX, CY = 535.4, 539.2, 320.1, 247.6


def read_frames(video, depth_npz):
    cap = cv2.VideoCapture(video)
    colors = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        colors.append(frame)
    depths = np.load(depth_npz)['depth']
    n = min(len(colors), len(depths))
    if n == 0:
        raise RuntimeError('no frames in %s and %s' % (video, depth_npz))
    return list(zip(colors[:n], depths[:n]))


def image_msg(image_type, frame, encoding):
    msg = image_type()
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = frame.strides[0]
    msg.data = frame.tobytes()
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('video', nargs='?', default=os.path.join(TEST_DIR, 'rgbd_walking.mp4'))
    parser.add_argument('depth', nargs='?', default=os.path.join(TEST_DIR, 'rgbd_walking_depth.npz'))
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--frame-id', default='camera_rgb_optical_frame')
    # without the arguments of ROS (__name:=..., __log:=... of roslaunch, --ros-args ... of ROS 2)
    argv = sys.argv[1:sys.argv.index('--ros-args')] if '--ros-args' in sys.argv else sys.argv[1:]
    args = parser.parse_args([a for a in argv if ':=' not in a])
    frames = read_frames(args.video, args.depth)
    k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]

    if os.environ.get('ROS_VERSION') == '1':
        import rospy
        from sensor_msgs.msg import CameraInfo, Image
        rospy.init_node('publish_rgbd')
        publishers = [rospy.Publisher(name, msg_type, queue_size=10)
                      for name, msg_type in [('image', Image), ('depth', Image), ('camera_info', CameraInfo)]]
        info = CameraInfo(width=640, height=480, distortion_model='plumb_bob', D=[0.0] * 5, K=k, R=r, P=p)
        now = rospy.Time.now
    else:
        import rclpy
        from sensor_msgs.msg import CameraInfo, Image
        rclpy.init()
        node = rclpy.create_node('publish_rgbd')
        publishers = [node.create_publisher(msg_type, name, 10)
                      for name, msg_type in [('image', Image), ('depth', Image), ('camera_info', CameraInfo)]]
        info = CameraInfo(width=640, height=480, distortion_model='plumb_bob', d=[0.0] * 5, k=k, r=r, p=p)

        def now():
            return node.get_clock().now().to_msg()

    state = {'index': 0}

    def publish(*_):
        color, depth = frames[state['index'] % len(frames)]
        messages = [image_msg(Image, color, 'bgr8'), image_msg(Image, depth, '16UC1'), info]
        stamp = now()
        for msg, publisher in zip(messages, publishers):
            msg.header.stamp = stamp
            msg.header.frame_id = args.frame_id
            publisher.publish(msg)
        state['index'] += 1

    if os.environ.get('ROS_VERSION') == '1':
        rospy.Timer(rospy.Duration(1.0 / args.rate), publish)
        rospy.spin()
    else:
        node.create_timer(1.0 / args.rate, publish)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
