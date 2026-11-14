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


# Dump the interface of each node of opencv_apps (ROS2) to doc/nodes.json, from which doc/conf.py generates the
# pages of the nodes: run each node, subscribe its outputs (the nodes subscribe their inputs only when their
# outputs are subscribed), and get its publishers, subscriptions, services and parameters.
#   source install/setup.bash; python3 doc/tools/dump_nodes.py

import json
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters
import rclpy
from rosidl_runtime_py.utilities import get_message

DOC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE_DIR = os.path.dirname(DOC_DIR)
# the parameters of every ROS2 node
COMMON_PARAMETERS = ['use_sim_time', 'start_type_description_service', 'qos_overrides']
PARAMETER_TYPES = {1: 'bool', 2: 'int', 3: 'double', 4: 'string', 5: 'byte[]', 6: 'bool[]', 7: 'int[]', 8: 'double[]',
                   9: 'string[]'}


def parameter_value(value):
    for field in ['bool_value', 'integer_value', 'double_value', 'string_value', 'byte_array_value',
                  'bool_array_value', 'integer_array_value', 'double_array_value', 'string_array_value']:
        if value.type == {'bool_value': 1, 'integer_value': 2, 'double_value': 3, 'string_value': 4,
                          'byte_array_value': 5, 'bool_array_value': 6, 'integer_array_value': 7,
                          'double_array_value': 8, 'string_array_value': 9}[field]:
            v = getattr(value, field)
            return list(v) if field.endswith('array_value') else v
    return None


def call(node, client, request, timeout=10.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError('no service ' + client.srv_name)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError('no response of ' + client.srv_name)
    return future.result()


def node_names(node):
    rclpy.spin_once(node, timeout_sec=0.2)
    return set(node.get_node_names_and_namespaces()) - {(node.get_name(), '/')}


def dump(node, executable):
    # the nodes before starting the executable, so that its node is not taken as an existing one
    existing = node_names(node)
    # in its own process group, to stop the node with ros2 run
    process = subprocess.Popen(['ros2', 'run', 'opencv_apps', executable, '--ros-args', '-p', 'debug_view:=false'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        # the node of the executable, which did not exist before
        name = None
        end_time = time.time() + 30
        while name is None and time.time() < end_time:
            names = node_names(node) - existing
            name = sorted(names)[0] if names else None
        if name is None:
            raise RuntimeError('no node of ' + executable)
        node_name, namespace = name
        full_name = (namespace.rstrip('/') + '/' + node_name)
        time.sleep(1.0)
        publishers = node.get_publisher_names_and_types_by_node(node_name, namespace)
        # subscribe the image outputs and the outputs of the node (e.g. ~/detections of detection_3d, which has no
        # image output), so that the node subscribes its inputs
        subscriptions = []
        for topic, types in publishers:
            if types[0] == 'sensor_msgs/msg/Image' or topic.startswith(full_name + '/'):
                subscriptions.append(node.create_subscription(get_message(types[0]), topic, lambda msg: None, 1))
        end_time = time.time() + 5
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.2)
        inputs = node.get_subscriber_names_and_types_by_node(node_name, namespace)
        services = node.get_service_names_and_types_by_node(node_name, namespace)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)

        names = call(node, node.create_client(ListParameters, full_name + '/list_parameters'),
                     ListParameters.Request()).result.names
        names = sorted(n for n in names if n.split('.')[0] not in COMMON_PARAMETERS)
        descriptors = call(node, node.create_client(DescribeParameters, full_name + '/describe_parameters'),
                           DescribeParameters.Request(names=names)).descriptors
        values = call(node, node.create_client(GetParameters, full_name + '/get_parameters'),
                      GetParameters.Request(names=names)).values
        parameters = []
        for descriptor, value in zip(descriptors, values):
            parameter = {'name': descriptor.name, 'type': PARAMETER_TYPES.get(descriptor.type, str(descriptor.type)),
                         'default': parameter_value(value), 'description': descriptor.description}
            for r in list(descriptor.integer_range) + list(descriptor.floating_point_range):
                parameter['range'] = [r.from_value, r.to_value]
            parameters.append(parameter)

        def topics(names_and_types, exclude):
            return [{'name': t.replace(full_name + '/', '~/'), 'type': types[0]} for t, types in names_and_types
                    if not any(t.endswith(e) for e in exclude)]
        return {'node': node_name,
                'publishers': topics(publishers, ['/rosout', '/parameter_events']),
                'subscriptions': topics(inputs, ['/parameter_events']),
                'services': topics(services, ['_parameters', '_parameter_types', 'get_type_description',
                                              'set_parameters_atomically']),
                'parameters': parameters}
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        # wait for the node to disappear from the graph
        end_time = time.time() + 10
        while node_names(node) - existing and time.time() < end_time:
            pass


def main():
    plugins = ET.parse(os.path.join(PACKAGE_DIR, 'nodelet_plugins.xml')).getroot()
    descriptions = {c.get('name').split('/')[-1]: ' '.join(c.find('description').text.split())
                    for c in plugins.iter('class') if c.find('description') is not None}
    executables = sorted(f[:-len('.launch.xml')] for f in os.listdir(os.path.join(PACKAGE_DIR, 'launch'))
                         if f.endswith('.launch.xml'))
    rclpy.init()
    node = rclpy.create_node('dump_nodes')
    nodes = {}
    failed = []
    for executable in executables:
        try:
            nodes[executable] = dump(node, executable)
            nodes[executable]['description'] = descriptions.get(executable, '')
            print('%-40s %d publishers, %d subscriptions, %d parameters' % (
                executable, len(nodes[executable]['publishers']), len(nodes[executable]['subscriptions']),
                len(nodes[executable]['parameters'])), flush=True)
        except RuntimeError as e:
            print('%-40s %s' % (executable, e), flush=True)
            failed.append(executable)
    rclpy.shutdown()
    if failed:
        # keep the previous nodes.json, as the documentation needs all the nodes
        sys.exit('failed to dump %s, nodes.json is not updated' % ', '.join(failed))
    with open(os.path.join(DOC_DIR, 'nodes.json'), 'w') as f:
        json.dump(nodes, f, indent=1, sort_keys=True)
        f.write('\n')


if __name__ == '__main__':
    main()
