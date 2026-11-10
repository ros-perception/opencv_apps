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
#  * Neither the name of the Kei Okada nor the names of its
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

# message_filters with the ROS2 API, on the ROS1 message_filters which this module shadows on sys.path:
# Subscriber(node, msg_type, topic) instead of Subscriber(topic, msg_type).

import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
_path = [p for p in sys.path if os.path.abspath(p or '.') != _here]
try:
    import importlib.machinery
    import importlib.util
    _spec = importlib.machinery.PathFinder.find_spec('message_filters', _path)
    _ros1 = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_ros1)
except ImportError:  # python2
    import imp
    _ros1 = imp.load_module('_ros1_message_filters', *imp.find_module('message_filters', _path))

TimeSynchronizer = _ros1.TimeSynchronizer
ApproximateTimeSynchronizer = _ros1.ApproximateTimeSynchronizer
Cache = _ros1.Cache


class Subscriber(_ros1.Subscriber):
    def __init__(self, node, msg_type, topic, qos_profile=None, **kwargs):
        super(Subscriber, self).__init__(topic, msg_type)
