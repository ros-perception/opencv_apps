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

"""Publish the raw data of a LiDAR of HDMapping / Mandeye (lidarNNNN.laz and imuNNNN.csv), e.g. HUMANOID-LIO-DATASET.

The points of the .laz (in the frame of the LiDAR, with the time of each point) are published as the scans of the
period (sensor_msgs/PointCloud2, x y z, on points, in --frame-id), stamped with the current time at the same rate
(or --speed times), and the static TF of --base-frame, whose z is up by the gravity of the IMU at the start (the mean
of the accelerations of the first second) and whose x is the x of the LiDAR on the floor, at the LiDAR. ROS 2, with
laspy and lazrs:

  pip install laspy lazrs
  # the Livox Mid-360 of a Unitree G1 walking in a room (CC BY 4.0, https://zenodo.org/records/21318128)
  wget https://zenodo.org/records/21318128/files/flat-room.7z && 7z x flat-room.7z
  python3 publish_hdmapping.py flat-room/trial-1/hdmapping-raw
"""

import argparse
import glob
import math
import os
import sys
import time

import numpy as np


def scans(directory, period):
    """Yield the points (N x 3, float32) of each period of the .laz files of the directory, in the order of time."""
    import laspy
    for path in sorted(glob.glob(os.path.join(directory, 'lidar*.laz'))):
        data = laspy.read(path)
        points = np.stack([data.x, data.y, data.z], 1).astype(np.float32)
        stamps = np.asarray(data.gps_time)
        valid = np.linalg.norm(points, axis=1) > 0.05  # the points of no return are at the origin
        points, stamps = points[valid], stamps[valid]
        order = np.argsort(stamps, kind='stable')
        points, stamps = points[order], stamps[order]
        edges = np.arange(math.floor(stamps[0] / period) * period, stamps[-1] + period, period)
        indices = np.searchsorted(stamps, edges)
        for first, last in zip(indices[:-1], indices[1:]):
            if last - first > 100:
                yield points[first:last]


def base_rotation(directory):
    """Return the rotation of the LiDAR in the base frame (z up by the gravity of the IMU at the start)."""
    path = sorted(glob.glob(os.path.join(directory, 'imu*.csv')))[0]
    imu = np.loadtxt(path, skiprows=1, usecols=(0, 4, 5, 6))
    acceleration = imu[imu[:, 0] < imu[0, 0] + 1e9, 1:4].mean(axis=0)  # of the first second, the IMU at rest
    z = acceleration / np.linalg.norm(acceleration)  # the up of the specific force, in the frame of the LiDAR
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])  # the rows: the axes of the base in the frame of the LiDAR


def quaternion(rotation):
    """Return the quaternion (x, y, z, w) of a rotation matrix, from its largest component."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * math.sqrt(1.0 + trace)
        return (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4.0
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        return s / 4.0, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s
    if m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 - m[0, 0] + m[1, 1] - m[2, 2])
        return (m[0, 1] + m[1, 0]) / s, s / 4.0, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s
    s = 2.0 * math.sqrt(1.0 - m[0, 0] - m[1, 1] + m[2, 2])
    return (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4.0, (m[1, 0] - m[0, 1]) / s


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', help='the directory of lidarNNNN.laz and imuNNNN.csv')
    parser.add_argument('--period', type=float, default=0.1, help='period (s) of a scan')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--frame-id', default='lidar')
    parser.add_argument('--base-frame', default='base_link')
    argv = sys.argv[1:sys.argv.index('--ros-args')] if '--ros-args' in sys.argv else sys.argv[1:]
    args = parser.parse_args(argv)

    from geometry_msgs.msg import TransformStamped
    import rclpy
    from sensor_msgs.msg import PointCloud2, PointField
    import tf2_ros
    rclpy.init()
    node = rclpy.create_node('publish_hdmapping')
    publisher = node.create_publisher(PointCloud2, 'points', 10)
    static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = args.base_frame
    transform.child_frame_id = args.frame_id
    # the LiDAR in the base: the transpose of the axes of the base in the LiDAR
    (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
     transform.transform.rotation.w) = quaternion(base_rotation(args.directory))
    static_broadcaster.sendTransform(transform)

    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    next_time = time.monotonic()
    for points in scans(args.directory, args.period):
        if not rclpy.ok():
            break
        msg = PointCloud2(height=1, width=len(points), is_bigendian=sys.byteorder == 'big', point_step=12,
                          row_step=12 * len(points), is_dense=True, data=points.tobytes(), fields=fields)
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = args.frame_id
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        next_time += args.period / args.speed
        time.sleep(max(0.0, next_time - time.monotonic()))
    node.get_logger().info('published all the scans')


if __name__ == '__main__':
    main()
