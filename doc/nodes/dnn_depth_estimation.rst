dnn_depth_estimation
====================

Monocular depth estimation with a DNN model, e.g. MiDaS.

Monocular depth estimation, by default with MiDaS v2.1 small, whose output is the relative inverse depth (larger is nearer, without the scale and the shift), or with a model of the same input and output, or with a model of the depth in meters (metric_depth), e.g. Depth Anything V2 Metric, whose ~/depth is the depth image of detection_3d.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Depth and 3D localization, the nodes of which publish ``~/inverse_depth`` or ``~/depth`` (Image) of dnn_depth_estimation, and ``~/detections`` (vision_msgs/Detection3DArray), ``~/markers`` (for rviz) and the TF frames of detection_3d

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_depth_estimation.gif
   :alt: demo of dnn_depth_estimation

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_depth_estimation.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_depth_estimation.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/keybridge.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node keybridge.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_depth_estimation.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_depth_estimation/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher keybridge.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_depth_estimation.launch image:=camera/image_raw

See :doc:`../credits` for the license of the video.

Models
------

The node runs MiDaS v2.1 small by default, whose output is the relative inverse depth (``~/inverse_depth``), or a model of the depth in meters (``metric_depth:=true``, ``~/depth``), e.g. Depth Anything V2 Metric, for detection_3d with a monocular camera. Depth Anything V2 Metric Small is converted to onnx readable by OpenCV 4.10 and later (by tools/export_depth_anything_metric.py) in `k-okada/depth-anything-v2-metric-small-onnx <https://huggingface.co/k-okada/depth-anything-v2-metric-small-onnx>`_ (Apache-2.0).

MiDaS v2.1 small (default)
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_depth_estimation.gif
   :alt: demo of dnn_depth_estimation with midas

.. code-block:: bash

   ros2 launch opencv_apps dnn_depth_estimation.launch.xml image:=image

MiDaS v2.1 small (256x256): fast on a CPU, and read by OpenCV 4 and 5; the relative inverse depth, larger nearer, without the scale in meters.

Depth Anything V2 Metric Small, indoors (Hypersim)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_depth_estimation_hypersim.gif
   :alt: demo of dnn_depth_estimation with hypersim

.. code-block:: bash

   ros2 launch opencv_apps dnn_depth_estimation.launch.xml image:=image input_height:=518 input_width:=518 metric_depth:=true model:=https://huggingface.co/k-okada/depth-anything-v2-metric-small-onnx/resolve/5494458701c7274fa82af484e2f2d226c948b54d/depth_anything_v2_metric_hypersim_vits_518.onnx

Depth Anything V2 Metric Small trained on Hypersim (518x518, 99 MB): the depth in meters indoors, up to about 20 m, e.g. for detection_3d in a room; needs OpenCV 4.10 or later, and a GPU to run it in real time. The demo is the RGB-D sequence of TUM (its color images only).

Depth Anything V2 Metric Small, outdoors (Virtual KITTI)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_depth_estimation_vkitti.gif
   :alt: demo of dnn_depth_estimation with vkitti

.. code-block:: bash

   ros2 launch opencv_apps dnn_depth_estimation.launch.xml image:=image input_height:=518 input_width:=518 metric_depth:=true model:=https://huggingface.co/k-okada/depth-anything-v2-metric-small-onnx/resolve/5494458701c7274fa82af484e2f2d226c948b54d/depth_anything_v2_metric_vkitti_vits_518.onnx

Depth Anything V2 Metric Small trained on Virtual KITTI (518x518, 99 MB): the depth in meters outdoors, up to about 80 m, e.g. on the road; needs OpenCV 4.10 or later.

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``image``
     - ``sensor_msgs/msg/Image``

Published topics
----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
   * - ``~/inverse_depth``
     - ``sensor_msgs/msg/Image``

Parameters
----------

See also the :ref:`common parameters <common-parameters>`.

.. list-table::
   :header-rows: 1

   * - Name
     - Type
     - Default (range)
     - Changed at run time
     - Description
   * - ``backend``
     - string
     - "auto"
     - yes
     - Backend of cv::dnn: auto, cpu, opencl, opencl_fp16, cuda or cuda_fp16. auto uses CUDA when OpenCV is built with CUDA and a CUDA device is found, and the CPU otherwise.
   * - ``input_height``
     - int
     - 256 (1 - 4096)
     - yes
     - Height of the network input, 256 for MiDaS v2.1 small
   * - ``input_width``
     - int
     - 256 (1 - 4096)
     - yes
     - Width of the network input, 256 for MiDaS v2.1 small
   * - ``metric_depth``
     - bool
     - false
     - no, restart the node
     - The output of the model is the depth in meters (e.g. Depth Anything V2 Metric), published on ~/depth (32FC1), e.g. for the depth input of detection_3d, instead of the relative inverse depth on ~/inverse_depth (MiDaS)
   * - ``model``
     - string
     - ""
     - yes
     - Path or URL of the model (onnx) of the same input and output as MiDaS, empty for MiDaS v2.1 small. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers

Source
------

`src/nodelet/dnn_depth_estimation_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_depth_estimation_nodelet.cpp>`_, 
`launch/dnn_depth_estimation.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_depth_estimation.launch.xml>`_
