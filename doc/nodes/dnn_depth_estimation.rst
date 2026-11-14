dnn_depth_estimation
====================

Monocular depth estimation with a DNN model, e.g. MiDaS.

Monocular depth estimation, by default with MiDaS v2.1 small, whose output is the relative inverse depth (larger is nearer, without the scale and the shift), or with a model of the same input and output.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Image filters, the nodes of which publish the processed image in ``~/image``

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
