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

# Publish the scans of a simulated LiDAR moving in a room with pillars, in a loop whose stamps start again
# at each loop (which restarts the odometry): points (sensor_msgs/PointCloud2) of the non-repetitive scanning of
# Livox Mid-360 (new random directions in its field of view, 360 x -7 to 52 degrees, at each scan) and scan
# (sensor_msgs/LaserScan of a 2D LiDAR of 360 degrees at the same pose), in the frame lidar, for test_laser_odometry.
# The sensor moves 2 m forward and 1 m left and turns 45 degrees (pose(), in the frame of the sensor at the start).
#   publish_livox_sim.py [--rate 10] [--frames 40]

import argparse
import math
import os
import sys

import numpy as np

# the room (inside) and the pillars (outside), as the boxes of their minimum and maximum corners, in the frame of the
# sensor at the start (x forward, z up, 0.5 m above the floor)
ROOM = (np.array([-3.0, -4.0, -0.5]), np.array([6.0, 4.0, 2.5]))
PILLARS = [(np.array([2.0, -2.5, -0.5]), np.array([2.6, -1.9, 2.5])),
           (np.array([4.0, 1.5, -0.5]), np.array([4.8, 2.1, 2.5])),
           (np.array([-1.5, 2.0, -0.5]), np.array([-0.9, 3.0, 1.0])),
           (np.array([0.5, -3.5, -0.5]), np.array([1.5, -3.0, 0.3]))]


def pose(t):
    """Return the 4x4 pose of the sensor at t in [0, 1], in the frame of the sensor at the start."""
    yaw = math.radians(45.0) * t
    transform = np.eye(4)
    transform[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    transform[:3, 3] = [2.0 * t, 1.0 * t * t, 0.0]
    return transform


def ranges(origin, directions):
    """Return the distances along the directions (N x 3) from the origin to the room or the pillars."""
    with np.errstate(divide='ignore', invalid='ignore'):
        inverse = 1.0 / directions
        # the exit of the inside of the room
        t1, t2 = (ROOM[0] - origin) * inverse, (ROOM[1] - origin) * inverse
        distance = np.min(np.maximum(t1, t2), axis=1)
        for low, high in PILLARS:
            t1, t2 = (low - origin) * inverse, (high - origin) * inverse
            near, far = np.max(np.minimum(t1, t2), axis=1), np.min(np.maximum(t1, t2), axis=1)
            hit = (near <= far) & (near > 0.0)
            distance = np.where(hit & (near < distance), near, distance)
    return distance


def livox_points(transform, rng, count=8000):
    """Return the points (N x 3, float32, in the frame of the sensor) of a scan of random directions of Mid-360."""
    azimuth = rng.uniform(-math.pi, math.pi, count)
    elevation = rng.uniform(math.radians(-7.0), math.radians(52.0), count)
    local = np.stack([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)], 1)
    distance = ranges(transform[:3, 3], local @ transform[:3, :3].T)
    distance = distance + rng.normal(0.0, 0.01, count)  # 1 cm of the noise of the range
    return (local * distance[:, None]).astype(np.float32)


def scan_ranges(transform, rng, count=720):
    """Return the ranges of a 2D scan of 360 degrees (angle_min -pi, angle_increment 2 pi / count)."""
    angle = -math.pi + np.arange(count) * 2.0 * math.pi / count
    local = np.stack([np.cos(angle), np.sin(angle), np.zeros(count)], 1)
    return (ranges(transform[:3, 3], local @ transform[:3, :3].T) + rng.normal(0.0, 0.01, count)).astype(np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--frames', type=int, default=40)
    argv = sys.argv[1:sys.argv.index('--ros-args')] if '--ros-args' in sys.argv else sys.argv[1:]
    args = parser.parse_args([a for a in argv if ':=' not in a])

    if os.environ.get('ROS_VERSION') == '1':
        import rospy
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        rospy.init_node('publish_livox_sim')
        points_pub = rospy.Publisher('points', PointCloud2, queue_size=10)
        scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
        start = rospy.Time.now()

        def stamp_of(seconds):
            return start + rospy.Duration.from_sec(seconds)
    else:
        import rclpy
        import rclpy.duration
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        rclpy.init()
        node = rclpy.create_node('publish_livox_sim')
        points_pub = node.create_publisher(PointCloud2, 'points', 10)
        scan_pub = node.create_publisher(LaserScan, 'scan', 10)
        start = node.get_clock().now()

        def stamp_of(seconds):
            return (start + rclpy.duration.Duration(seconds=seconds)).to_msg()
    rng = np.random.default_rng(0)
    state = {'index': 0}

    def publish(*_):
        i = state['index'] % args.frames
        transform = pose(i / (args.frames - 1))
        stamp = stamp_of(i / args.rate)
        points = livox_points(transform, rng)
        cloud = PointCloud2(height=1, width=len(points), is_bigendian=sys.byteorder == 'big', point_step=12,
                            row_step=12 * len(points), is_dense=True, data=points.tobytes(),
                            fields=[PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                                    for k, n in enumerate('xyz')])
        scan = LaserScan(angle_min=-math.pi, angle_max=math.pi - 2.0 * math.pi / 720,
                         angle_increment=2.0 * math.pi / 720, range_min=0.1, range_max=30.0,
                         ranges=scan_ranges(transform, rng).tolist())
        for msg, publisher in [(cloud, points_pub), (scan, scan_pub)]:
            msg.header.stamp = stamp
            msg.header.frame_id = 'lidar'
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
