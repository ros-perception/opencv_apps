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

"""Publish a sequence of the TUM RGB-D dataset (https://cvg.cit.tum.de/data/datasets/rgbd-dataset, CC BY 4.0).

The color images (bgr8, image), the depth images associated to them (16UC1 in mm, depth) and camera_info of the
camera of the sequence (freiburg1, 2 or 3 by its name), of the directory extracted from its .tgz, stamped with the
current time (or of the dataset, --original-stamps) at the same rate (or --speed times), and the static TF of the
optical frame of the camera (camera_rgb_optical_frame) on --base-frame (x forward and z up, at the camera). ROS 2:

  wget https://cvg.cit.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam.tgz
  tar xf rgbd_dataset_freiburg2_pioneer_slam.tgz
  python3 publish_tum_rgbd.py rgbd_dataset_freiburg2_pioneer_slam
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# the intrinsics (fx, fy, cx, cy) of the cameras of the dataset, of the color images; the depth images are registered
# to them, and the depth of freiburg1 and 2 is 5000 per meter
INTRINSICS = {'freiburg1': (517.3, 516.5, 318.6, 255.3), 'freiburg2': (520.9, 521.0, 325.1, 249.7),
              'freiburg3': (535.4, 539.2, 320.1, 247.6)}


def associate(directory):
    """Return the pairs of the files of the color images and the closest depth images (within 0.02 s)."""
    def read(name):
        with open(os.path.join(directory, name)) as f:
            return [(float(line.split()[0]), line.split()[1]) for line in f if not line.startswith('#')]
    depths = read('depth.txt')
    stamps = np.array([t for t, _ in depths])
    pairs = []
    for t, color in read('rgb.txt'):
        i = int(np.argmin(abs(stamps - t)))
        if abs(stamps[i] - t) < 0.02:
            pairs.append((t, color, depths[i][1]))
    return pairs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', help='the directory of a sequence, e.g. rgbd_dataset_freiburg2_pioneer_slam')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--frame-id', default='camera_rgb_optical_frame')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--original-stamps', action='store_true',
                        help='the stamps of the dataset, e.g. to compare the odometry with its ground truth')
    argv = sys.argv[1:sys.argv.index('--ros-args')] if '--ros-args' in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)
    camera = next((c for c in INTRINSICS if c in os.path.basename(os.path.normpath(args.directory))), 'freiburg1')
    fx, fy, cx, cy = INTRINSICS[camera]

    from geometry_msgs.msg import TransformStamped
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image
    import tf2_ros
    rclpy.init()
    node = rclpy.create_node('publish_tum_rgbd')
    image_pub = node.create_publisher(Image, 'image', 10)
    depth_pub = node.create_publisher(Image, 'depth', 10)
    info_pub = node.create_publisher(CameraInfo, 'camera_info', 10)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    optical = TransformStamped()
    optical.header.stamp = node.get_clock().now().to_msg()
    optical.header.frame_id = args.base_frame
    optical.child_frame_id = args.frame_id
    (optical.transform.rotation.x, optical.transform.rotation.y, optical.transform.rotation.z,
     optical.transform.rotation.w) = (-0.5, 0.5, -0.5, 0.5)
    static_broadcaster.sendTransform(optical)

    def image_msg(frame, encoding):
        msg = Image(height=frame.shape[0], width=frame.shape[1], encoding=encoding, is_bigendian=0,
                    step=frame.strides[0], data=frame.tobytes())
        msg.header.frame_id = args.frame_id
        return msg

    pairs = associate(args.directory)
    start_wall, start_data = time.monotonic(), pairs[0][0]
    for t, color_file, depth_file in pairs:
        if not rclpy.ok():
            break
        time.sleep(max(0.0, start_wall + (t - start_data) / args.speed - time.monotonic()))
        color = cv2.imread(os.path.join(args.directory, color_file))
        depth = (cv2.imread(os.path.join(args.directory, depth_file), cv2.IMREAD_UNCHANGED) / 5.0).astype(np.uint16)
        stamp = node.get_clock().now().to_msg()
        if args.original_stamps:
            stamp.sec, stamp.nanosec = int(t), int(round((t % 1.0) * 1e9)) % 1000000000
        info = CameraInfo(width=color.shape[1], height=color.shape[0], distortion_model='plumb_bob', d=[0.0] * 5,
                          k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0], r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                          p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        info.header.frame_id = args.frame_id
        messages = [(image_msg(color, 'bgr8'), image_pub), (image_msg(depth, '16UC1'), depth_pub), (info, info_pub)]
        for msg, publisher in messages:
            msg.header.stamp = stamp
            publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
    node.get_logger().info('published all the images')


if __name__ == '__main__':
    main()
