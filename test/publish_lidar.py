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

# Publish a LiDAR-camera sequence in a loop: image (bgr8), camera_info, points (sensor_msgs/PointCloud2, x y z in the
# LiDAR frame) with the stamp of the image, and the static TF of the extrinsics from the camera frame to the LiDAR
# frame, from the color video and the points in .npz (points: all the points, counts: the number of points of each
# image, K: the camera matrix, T_camera_lidar: the 4x4 transform of the points to the camera frame), by default
# test/lidar_boreas.mp4 and test/lidar_boreas_points.npz: 20 images (every second of 40, 612x512, 1/4 of the camera)
# and the points in the view of the camera (every third, float16) of the sequence boreas-2021-09-02-11-42 of the
# Boreas dataset (CC BY 4.0, https://www.boreas.utias.utoronto.ca/).
#   publish_lidar.py [VIDEO POINTS_NPZ] [--rate 5]

import argparse
import math
import os
import sys

import cv2
import numpy as np

TEST_DIR = os.path.dirname(os.path.abspath(__file__))


def read_frames(video, points_npz):
    cap = cv2.VideoCapture(video)
    colors = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        colors.append(frame)
    data = np.load(points_npz)
    clouds = np.split(data['points'].astype(np.float32), np.cumsum(data['counts'])[:-1])
    n = min(len(colors), len(clouds))
    if n == 0:
        raise RuntimeError('no frames in %s and %s' % (video, points_npz))
    return list(zip(colors[:n], clouds[:n])), data['K'], data['T_camera_lidar']


def quaternion(rotation):
    """Return the quaternion (x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + rotation[0, 0] + rotation[1, 1] + rotation[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) / 2.0,
                      rotation[2, 1] - rotation[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - rotation[0, 0] + rotation[1, 1] - rotation[2, 2])) / 2.0,
                      rotation[0, 2] - rotation[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - rotation[0, 0] - rotation[1, 1] + rotation[2, 2])) / 2.0,
                      rotation[1, 0] - rotation[0, 1])
    return x, y, z, w


def image_msg(image_type, frame):
    msg = image_type()
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = frame.strides[0]
    msg.data = frame.tobytes()
    return msg


def cloud_msg(cloud_type, field_type, points):
    msg = cloud_type()
    msg.height, msg.width = 1, len(points)
    msg.fields = [field_type(name=name, offset=4 * i, datatype=field_type.FLOAT32, count=1)
                  for i, name in enumerate('xyz')]
    msg.is_bigendian = sys.byteorder == 'big'
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, np.float32).tobytes()
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('video', nargs='?', default=os.path.join(TEST_DIR, 'lidar_boreas.mp4'))
    parser.add_argument('points', nargs='?', default=os.path.join(TEST_DIR, 'lidar_boreas_points.npz'))
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--camera-frame', default='camera')
    parser.add_argument('--lidar-frame', default='lidar')
    # without the arguments of ROS (__name:=..., __log:=... of roslaunch, --ros-args ... of ROS 2)
    argv = sys.argv[1:sys.argv.index('--ros-args')] if '--ros-args' in sys.argv else sys.argv[1:]
    args = parser.parse_args([a for a in argv if ':=' not in a])
    frames, k, t_camera_lidar = read_frames(args.video, args.points)
    height, width = frames[0][0].shape[:2]
    k = [float(v) for v in k.flatten()]
    p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]
    r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    if os.environ.get('ROS_VERSION') == '1':
        import rospy
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
        import tf2_ros
        rospy.init_node('publish_lidar')
        publishers = [rospy.Publisher(name, msg_type, queue_size=10)
                      for name, msg_type in [('image', Image), ('camera_info', CameraInfo), ('points', PointCloud2)]]
        info = CameraInfo(width=width, height=height, distortion_model='plumb_bob', D=[0.0] * 5, K=k, R=r, P=p)
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        now = rospy.Time.now
    else:
        from geometry_msgs.msg import TransformStamped
        import rclpy
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
        import tf2_ros
        rclpy.init()
        node = rclpy.create_node('publish_lidar')
        publishers = [node.create_publisher(msg_type, name, 10)
                      for name, msg_type in [('image', Image), ('camera_info', CameraInfo), ('points', PointCloud2)]]
        info = CameraInfo(width=width, height=height, distortion_model='plumb_bob', d=[0.0] * 5, k=k, r=r, p=p)
        static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

        def now():
            return node.get_clock().now().to_msg()

    # the extrinsics: the transform of the points from the LiDAR frame to the camera frame
    transform = TransformStamped()
    transform.header.stamp = now()
    transform.header.frame_id = args.camera_frame
    transform.child_frame_id = args.lidar_frame
    (transform.transform.translation.x, transform.transform.translation.y,
     transform.transform.translation.z) = [float(v) for v in t_camera_lidar[:3, 3]]
    (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
     transform.transform.rotation.w) = quaternion(t_camera_lidar[:3, :3])
    static_broadcaster.sendTransform(transform)

    state = {'index': 0}

    def publish(*_):
        color, points = frames[state['index'] % len(frames)]
        messages = [image_msg(Image, color), info, cloud_msg(PointCloud2, PointField, points)]
        stamp = now()
        for msg, frame_id, publisher in zip(messages, [args.camera_frame, args.camera_frame, args.lidar_frame],
                                            publishers):
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
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
