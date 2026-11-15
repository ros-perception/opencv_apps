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

# the odometry of laser_odometry in the simulated scans of test/publish_livox_sim.py, which moves 2 m forward and 1 m
# left and turns 45 degrees in a room with pillars: the points of the non-repetitive scanning of Livox Mid-360
# (input_type points) and the 2D scan (input_type scan, laser_odometry_scan); the pose at the last scan is within
# 5 cm and 2 degrees of the truth

import math
import os
import sys
import unittest

import launch.actions
from nav_msgs.msg import Odometry
from opencv_apps_test import description, include, MessagesTest, TEST_DIR
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_livox_sim.py')],
                                      output='screen'),
        include('laser_odometry', points='points', voxel_size='0.2', max_range='20.0'),
        include('laser_odometry', node_name='laser_odometry_scan', scan='scan', input_type='scan', voxel_size='0.1',
                max_range='20.0', publish_tf='false'),
    )


def at_end(msg):
    # the sensor at the last scan, 2 m forward, 1 m left and turned 45 degrees
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.degrees(2.0 * math.atan2(q.z, q.w))
    return (msg.header.frame_id == 'odom' and msg.child_frame_id == 'lidar' and
            math.hypot(p.x - 2.0, p.y - 1.0) < 0.05 and abs(p.z) < 0.05 and abs(yaw - 45.0) < 2.0)


class TestLaserOdometry(MessagesTest, unittest.TestCase):

    TOPICS = ['laser_odometry/odom', 'laser_odometry_scan/odom']
    CHECKS = {'laser_odometry/odom': (Odometry, at_end), 'laser_odometry_scan/odom': (Odometry, at_end)}
