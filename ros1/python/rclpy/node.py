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

import sys

import rospy


class Parameter(object):
    def __init__(self, name, value):
        self.name = name
        self.value = value


class Future(object):
    def __init__(self):
        self._result = None
        self._done = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def set_result(self, result):
        self._result = result
        self._done = True


class Client(object):
    def __init__(self, srv_type, srv_name):
        self.srv_name = srv_name
        self.proxy = rospy.ServiceProxy(srv_name, srv_type)

    def wait_for_service(self, timeout_sec=None):
        try:
            rospy.wait_for_service(self.srv_name, timeout_sec)
            return True
        except rospy.ROSException:
            return False

    def service_is_ready(self):
        return self.wait_for_service(0.001)

    def call(self, request):
        return self.proxy(request)

    def call_async(self, request):
        # rospy has no asynchronous call: call it here, and return a done future
        future = Future()
        future.set_result(self.proxy(request))
        return future


def _add_request_response_classes():
    # ROS2 service types have Request/Response, ROS1 ones have _request_class/_response_class
    for name, module in list(sys.modules.items()):
        if module is None or not name.endswith('.srv'):
            continue
        for srv in list(vars(module).values()):
            if isinstance(srv, type) and hasattr(srv, '_request_class') and not hasattr(srv, 'Request'):
                srv.Request = srv._request_class
                srv.Response = srv._response_class


class Node(object):
    def __init__(self, node_name, **kwargs):
        rospy.init_node(node_name)
        _add_request_response_classes()

    def get_name(self):
        return rospy.get_name().split('/')[-1]

    def declare_parameter(self, name, value=None, descriptor=None):
        # node parameters are private ones on ROS1
        return Parameter(name, rospy.get_param('~' + name, value))

    def get_parameter(self, name):
        return Parameter(name, rospy.get_param('~' + name))

    def create_client(self, srv_type, srv_name, **kwargs):
        return Client(srv_type, srv_name)

    def create_publisher(self, msg_type, topic, qos_profile=10, **kwargs):
        return rospy.Publisher(topic, msg_type, queue_size=qos_profile if isinstance(qos_profile, int) else 10)

    def create_subscription(self, msg_type, topic, callback, qos_profile=10, **kwargs):
        return rospy.Subscriber(topic, msg_type, callback)

    def get_logger(self):
        return _Logger()

    def destroy_node(self):
        pass


class _Logger(object):
    def debug(self, msg):
        rospy.logdebug(msg)

    def info(self, msg):
        rospy.loginfo(msg)

    def warn(self, msg):
        rospy.logwarn(msg)

    warning = warn

    def error(self, msg):
        rospy.logerr(msg)

    def fatal(self, msg):
        rospy.logfatal(msg)
