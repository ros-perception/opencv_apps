dnn_tracking
============

Track the target with the DNN based trackers of OpenCV (DaSiamRPN, NanoTrack, VitTrack).

This is a demo of the DNN based trackers of OpenCV: DaSiamRPN, NanoTrack and VitTrack. Select the target with mouse, or call ~set_roi service, as the tracking node.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later; DaSiamRPN on ROS One, jazzy and kilted (OpenCV 4.6), and also NanoTrack and VitTrack on lyrical (OpenCV 4.10)
   * - OpenCV
     - OpenCV 4.5.4 or later for DaSiamRPN, 4.7 or later for NanoTrack and 4.9 or later for VitTrack

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_tracking.gif
   :alt: demo of dnn_tracking

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_tracking.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_tracking.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_canal.mp4
   ffmpeg -i tos_canal.mp4 -frames:v 1 frame0.png  # the first frame
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_canal.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_tracking.launch.xml image:=image
   # terminal 3: then run once
   ros2 run opencv_apps set_image.py 300 160 160 220 frame0.png --ros-args -r set_roi:=/dnn_tracking/set_roi
   # terminal 4: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_tracking/image

Set the man on the first frame with set_image.py just after the video starts (the demo publishes the frames one by one and sets it at the start of each loop).

See :doc:`../credits` for the license of the video.

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
   * - ``~/output``
     - :ref:`opencv_apps/msg/RectArrayStamped <msg-RectArrayStamped>`

Services
--------

.. list-table::
   :header-rows: 1

   * - Service
     - Type
   * - ``~/set_roi``
     - ``opencv_apps/srv/SetROI``

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
   * - ``dasiamrpn_kernel_cls1``
     - string
     - ""
     - yes
     - dasiamrpn_kernel_cls1.onnx of DaSiamRPN, a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default https://www.dropbox.com/s/qvmtszx5h339a0w/dasiamrpn_kernel_cls1.onnx?dl=1
   * - ``dasiamrpn_kernel_r1``
     - string
     - ""
     - yes
     - dasiamrpn_kernel_r1.onnx of DaSiamRPN, a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default https://www.dropbox.com/s/999cqx5zrfi7w4p/dasiamrpn_kernel_r1.onnx?dl=1
   * - ``dasiamrpn_model``
     - string
     - ""
     - yes
     - dasiamrpn_model.onnx of DaSiamRPN, a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default https://www.dropbox.com/s/rr1lk9355vzolqv/dasiamrpn_model.onnx?dl=1
   * - ``nanotrack_backbone``
     - string
     - ""
     - yes
     - Backbone model of NanoTrack, a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default https://raw.githubusercontent.com/HonglinChu/SiamTrackers/8fe6791fecf13b95e6f19770af8963a9fc98759b/NanoTrack/models/nanotrackv2/nanotrack_backbone_sim.onnx
   * - ``nanotrack_neckhead``
     - string
     - ""
     - yes
     - Neck and head model of NanoTrack, a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default https://raw.githubusercontent.com/HonglinChu/SiamTrackers/8fe6791fecf13b95e6f19770af8963a9fc98759b/NanoTrack/models/nanotrackv2/nanotrack_head_sim.onnx
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``tracking_algorithm``
     - int
     - -1 (-1 - 2)
     - yes
     - Tracking Algorithm, -1: the newest one of this OpenCV (VitTrack, NanoTrack or DaSiamRPN), 0: DaSiamRPN, 1: NanoTrack (OpenCV 4.7 or later), 2: VitTrack (OpenCV 4.9 or later)
   * - ``vittrack_model``
     - string
     - ""
     - yes
     - Model of VitTrack, a path or URL (downloaded to ~/.ros/opencv_apps/models/ at the first time), empty for the default https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/models/object_tracking_vittrack/object_tracking_vittrack_2023sep.onnx

Source
------

`src/nodelet/dnn_tracking_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_tracking_nodelet.cpp>`_, 
`launch/dnn_tracking.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_tracking.launch.xml>`_
