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

# Publish a synthetic rectified camera (ROS 2): camera_info of the rectified image, whose R (the rotation of the
# rectified frame from the optical frame, 10 degrees about y) is not the identity and P is not K, and a square of
# 0.4 m facing the rectified camera at OBJECT (in the optical frame camera), as the depth image of the rectified
# image (32FC1), the points (organized, in the optical frame, with the padding of the rows) and a skeleton person of
# its box with a keypoint nose at its center, for test_detection_3d_rectified.
#   publish_rectified.py [--rate 10]

import argparse
import math
import sys

import numpy as np

OBJECT = (0.3, -0.1, 2.0)
WIDTH, HEIGHT = 640, 480
K = [400.0, 0.0, 300.0, 0.0, 400.0, 220.0, 0.0, 0.0, 1.0]
P = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
ANGLE = math.radians(10.0)
R = [math.cos(ANGLE), 0.0, math.sin(ANGLE), 0.0, 1.0, 0.0, -math.sin(ANGLE), 0.0, math.cos(ANGLE)]


def scene():
    """Return the depth image, the points (N x N x 3, in the optical frame) and the box and the center in pixels."""
    rotation = np.array(R).reshape(3, 3)
    center = rotation @ np.array(OBJECT)  # in the rectified frame
    fx, cx, fy, cy = P[0], P[2], P[5], P[6]
    u0, v0 = fx * center[0] / center[2] + cx, fy * center[1] / center[2] + cy
    half = 0.2 * fx / center[2]  # half of the square in pixels
    depth = np.zeros((HEIGHT, WIDTH), np.float32)
    depth[int(round(v0 - half)):int(round(v0 + half)), int(round(u0 - half)):int(round(u0 + half))] = center[2]
    # the points of the square in the rectified frame, rotated to the optical frame
    s = np.linspace(-0.2, 0.2, 21)
    xs, ys = np.meshgrid(s, s)
    square = np.stack([center[0] + xs, center[1] + ys, np.full_like(xs, center[2])], axis=-1)
    points = (square.reshape(-1, 3) @ rotation).reshape(square.shape).astype(np.float32)
    return depth, points, (u0, v0, 2 * half, 2 * half)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rate', type=float, default=10.0)
    argv = sys.argv[1:sys.argv.index('--ros-args')] if '--ros-args' in sys.argv else sys.argv[1:]
    args = parser.parse_args([a for a in argv if ':=' not in a])

    from opencv_apps.msg import Keypoint, Skeleton, SkeletonArrayStamped
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    rclpy.init()
    node = rclpy.create_node('publish_rectified')
    info_pub = node.create_publisher(CameraInfo, 'camera_info', 10)
    depth_pub = node.create_publisher(Image, 'depth', 10)
    points_pub = node.create_publisher(PointCloud2, 'points', 10)
    skeletons_pub = node.create_publisher(SkeletonArrayStamped, 'skeletons', 10)

    depth, points, (u0, v0, width, height) = scene()
    info = CameraInfo(width=WIDTH, height=HEIGHT, distortion_model='plumb_bob', d=[0.0] * 5, k=K, r=R, p=P)
    depth_msg = Image(height=HEIGHT, width=WIDTH, encoding='32FC1', is_bigendian=0, step=WIDTH * 4,
                      data=depth.tobytes())
    # organized points of 12 bytes and 4 bytes of the padding at the end of each row
    rows, columns = points.shape[:2]
    padded = np.zeros((rows, columns * 12 + 4), np.uint8)
    padded[:, :columns * 12] = points.reshape(rows, -1).view(np.uint8)
    cloud = PointCloud2(height=rows, width=columns, is_bigendian=sys.byteorder == 'big', point_step=12,
                        row_step=columns * 12 + 4, is_dense=True, data=padded.tobytes(),
                        fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                for i, n in enumerate('xyz')])
    skeleton = Skeleton(label='person', id=-1, score=0.9)
    skeleton.rect.x, skeleton.rect.y, skeleton.rect.width, skeleton.rect.height = u0, v0, width, height
    skeleton.keypoints = [Keypoint(name='nose', x=u0, y=v0, visibility=1.0, presence=1.0)]

    def publish():
        stamp = node.get_clock().now().to_msg()
        skeletons = SkeletonArrayStamped(skeletons=[skeleton])
        for msg, publisher in [(info, info_pub), (depth_msg, depth_pub), (cloud, points_pub),
                               (skeletons, skeletons_pub)]:
            msg.header.stamp = stamp
            msg.header.frame_id = 'camera'
            publisher.publish(msg)

    node.create_timer(1.0 / args.rate, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
