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


# the warnings of the inputs of detection_3d (and of the other nodes) on /rosout: the input of no publisher, with the
# published topics of its type to remap it to, and no 3D detection with the inputs not received or not synchronized;
# and no warning of the optional inputs of no publisher, also remapped

import os
import sys
import time
import unittest

import launch.actions
import launch_ros.actions
from opencv_apps_test import description, include, TEST_DIR
import pytest
from opencv_apps.msg import DetectionArrayStamped
import rclpy
from rcl_interfaces.msg import Log

# the stamps of /other_stamp_detections, the detections of dnn_object_detection, are later than those of the depth by
# far more than the queues of the synchronizer (e.g. of the clock of another computer), which never synchronizes them;
# by a few seconds, the depth of the stamps of slow detections would come in time to be synchronized
OFFSET = 1000.0


@pytest.mark.launch_test
def generate_test_description():
    return description(
        launch.actions.ExecuteProcess(cmd=[sys.executable, os.path.join(TEST_DIR, 'publish_rgbd.py')], output='screen'),
        include('dnn_object_detection', image='/image'),
        # the depth of the wrong name, e.g. of a camera of no depth_registered
        include('detection_3d', node_name='detection_3d_wrong_depth', input='/dnn_object_detection/detections',
                depth='/depth_registered/image_raw', camera_info='/camera_info'),
        # the input of the other stamps than the depth, by the test below (e.g. of the clock of another computer)
        include('detection_3d', node_name='detection_3d_other_stamps', input='/other_stamp_detections',
                depth='/depth', camera_info='/camera_info'),
        # the optional inputs: ~/template remapped out of the namespace of the node, ~/mask, and mask of dnn_inpainting
        # (which does not read its model without the subscribers of its output)
        launch_ros.actions.Node(package='opencv_apps', executable='match_template', name='match_template_optional',
                                remappings=[('image', '/image'), ('~/template', '/templates')],
                                parameters=[{'debug_view': False}]),
        launch_ros.actions.Node(package='opencv_apps', executable='dnn_inpainting', name='dnn_inpainting_optional',
                                remappings=[('image', '/image')], parameters=[{'debug_view': False}]),
    )


class TestDetection3DWarnings(unittest.TestCase):

    TIMEOUT = 120.0
    # node: the parts of a warning of the node
    WARNINGS = {
        'detection_3d_wrong_depth': [
            ['no publisher of the input /depth_registered/image_raw (sensor_msgs/msg/Image): remap it to one of the '
             'published topics of the type', '/depth'],
            ['no 3D detection, as no message of depth /depth_registered/image_raw is received'],
        ],
        'detection_3d_other_stamps': [
            ['no 3D detection, as the inputs are received', 'but not synchronized by their stamps'],
        ],
    }

    def test_warnings(self):
        rclpy.init()
        try:
            node = rclpy.create_node('test_detection_3d_warnings')
            logs = []
            node.create_subscription(Log, '/rosout', lambda msg: logs.append(msg), 100)
            detections_pub = node.create_publisher(DetectionArrayStamped, '/other_stamp_detections', 10)

            def other_stamp(msg):
                nanoseconds = msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec + int(OFFSET * 10 ** 9)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(nanoseconds, 10 ** 9)
                detections_pub.publish(msg)
            node.create_subscription(DetectionArrayStamped, '/dnn_object_detection/detections', other_stamp, 10)

            def missing():
                return {name: [parts for parts in warnings if not any(
                    log.name == name and log.level == Log.WARN and all(p in log.msg for p in parts) for log in logs)]
                    for name, warnings in self.WARNINGS.items()}

            end_time = time.time() + self.TIMEOUT
            while time.time() < end_time and any(missing().values()):
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertEqual({name: parts for name, parts in missing().items() if parts}, {})
            # the warnings of no publisher are at 5 - 10 s of the nodes, which are up with the warnings above
            end_time = time.time() + 10.0
            while time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
            optional = ('match_template_optional', 'dnn_inpainting_optional')
            self.assertEqual([log.msg for log in logs
                              if log.name in optional and log.level == Log.WARN and 'no publisher' in log.msg], [])
        finally:
            rclpy.shutdown()
