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

"""Export the metric depth models of Depth Anything V2 Small to onnx that cv::dnn of OpenCV 4.10 or later reads.

The models of https://huggingface.co/depth-anything (Apache 2.0) are exported with a fixed input (1 x 3 x size x
size, opset 12, the TorchScript exporter), simplified by onnx-simplifier without fusing MatMul and Add into Gemm, and
the weights of Gemm are transposed to transB=1, as cv::dnn of OpenCV 4 reads neither the dynamic shapes nor the
functions of the newer exporter. The onnx of https://huggingface.co/k-okada/depth-anything-v2-metric-small-onnx are
made by this script. In a virtualenv:

  pip install torch torchvision onnx onnxsim opencv-python
  git clone https://github.com/DepthAnything/Depth-Anything-V2.git
  python3 export_depth_anything_metric.py Depth-Anything-V2 hypersim   # indoors, max 20 m
  python3 export_depth_anything_metric.py Depth-Anything-V2 vkitti     # outdoors, max 80 m
"""

import argparse
import os
import sys
import urllib.request

MAX_DEPTH = {'hypersim': 20.0, 'vkitti': 80.0}
NAME = {'hypersim': 'Hypersim', 'vkitti': 'VKITTI'}


def transpose_gemm(model):
    """Transpose the weights of Gemm of transB=0 to transB=1, which cv::dnn of OpenCV 4 takes."""
    from onnx import helper, numpy_helper
    initializers = {i.name: i for i in model.graph.initializer}
    for node in model.graph.node:
        if node.op_type != 'Gemm' or node.input[1] not in initializers:
            continue
        if any(a.name == 'transB' and a.i == 1 for a in node.attribute):
            continue
        weight = initializers[node.input[1]]
        transposed = numpy_helper.from_array(numpy_helper.to_array(weight).T.copy(), weight.name + '_t')
        model.graph.initializer.append(transposed)
        node.input[1] = transposed.name
        for attribute in [a for a in node.attribute if a.name == 'transB']:
            node.attribute.remove(attribute)
        node.attribute.append(helper.make_attribute('transB', 1))
    used = {i for node in model.graph.node for i in node.input}
    for initializer in [i for i in model.graph.initializer if i.name not in used]:
        model.graph.initializer.remove(initializer)
    return model


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('repository', help='the clone of https://github.com/DepthAnything/Depth-Anything-V2')
    parser.add_argument('dataset', choices=sorted(MAX_DEPTH), help='hypersim (indoors) or vkitti (outdoors)')
    parser.add_argument('--size', type=int, default=518, help='the input size, a multiple of 14')
    args = parser.parse_args()

    import onnx
    import onnxsim
    import torch
    sys.path.insert(0, os.path.join(args.repository, 'metric_depth'))
    from depth_anything_v2.dpt import DepthAnythingV2

    checkpoint = 'depth_anything_v2_metric_%s_vits.pth' % args.dataset
    if not os.path.exists(checkpoint):
        urllib.request.urlretrieve('https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-%s-Small/resolve/'
                                   'main/%s' % (NAME[args.dataset], checkpoint), checkpoint)
    model = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384],
                            max_depth=MAX_DEPTH[args.dataset])
    model.load_state_dict(torch.load(checkpoint, map_location='cpu'))
    model.eval()
    output = 'depth_anything_v2_metric_%s_vits_%d.onnx' % (args.dataset, args.size)
    torch.onnx.export(model, torch.randn(1, 3, args.size, args.size), output, opset_version=12,
                      input_names=['image'], output_names=['depth'], do_constant_folding=True, dynamo=False)
    simplified, ok = onnxsim.simplify(onnx.load(output), skipped_optimizers=['fuse_matmul_add_bias_into_gemm'])
    if not ok:
        sys.exit('failed to simplify %s' % output)
    model = transpose_gemm(simplified)
    onnx.checker.check_model(model)
    onnx.save(model, output)
    print('wrote', output)


if __name__ == '__main__':
    main()
