dnn_edge_detection
==================

Detect the edges with a DNN model, e.g. DexiNed.

Edge detection with DexiNed of opencv_zoo (MIT License), whose ~/image is the edges in mono8 as edge_detection, the probability of the edge of each pixel (0 - 255). The model needs OpenCV 4.10 or later (OpenCV 4.6 cannot read it).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.1.0 and later
   * - ROS
     - lyrical and later; not ROS One (OpenCV 4.6), jazzy (OpenCV 4.6), kilted (OpenCV 4.6); tested with OpenCV 4.12 and 5.1, not yet with OpenCV 4.10 of lyrical
   * - OpenCV
     - OpenCV 4.10 or later, which reads DexiNed (DequantizeLinear)
   * - Group
     - Image filters, the nodes of which publish the processed image in ``~/image``

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_edge_detection.gif
   :alt: demo of dnn_edge_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_edge_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_edge_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_edge_detection.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_edge_detection/image

DexiNed of opencv_zoo (MIT License), the same clip as edge_detection; needs OpenCV 4.10 or later.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_edge_detection.launch image:=camera/image_raw

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
     - 512 (1 - 4096)
     - yes
     - Height of the network input, 512 for DexiNed
   * - ``input_width``
     - int
     - 512 (1 - 4096)
     - yes
     - Width of the network input, 512 for DexiNed
   * - ``model``
     - string
     - ""
     - yes
     - Path or URL of the model (onnx) of the same input and output as DexiNed, empty for DexiNed of opencv_zoo (OpenCV 4.10 or later). The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers

Source
------

`src/nodelet/dnn_edge_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_edge_detection_nodelet.cpp>`_, 
`launch/dnn_edge_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_edge_detection.launch.xml>`_
