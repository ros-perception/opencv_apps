threshold
=========

Thresholds the image (``cv::threshold()``): binary, inverted binary, truncated, to zero or inverted to zero, optionally with the threshold of the Otsu's method.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11.14 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/threshold.gif
   :alt: demo of threshold

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps threshold.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps threshold.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps threshold.launch.xml image:=image apply_otsu:=true max_binary:=255 threshold:=127 threshold_type:=0
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /threshold/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps threshold.launch image:=camera/image_raw apply_otsu:=true max_binary:=255 threshold:=127 threshold_type:=0

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
   * - ``apply_otsu``
     - bool
     - false
     - yes
     - Apply otsu threshold
   * - ``max_binary``
     - int
     - 255 (0 - 255)
     - yes
     - Max Binary value
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``threshold``
     - int
     - 150 (0 - 255)
     - yes
     - Threshold value
   * - ``threshold_type``
     - int
     - 0 (0 - 4)
     - yes
     - Threshold Type, 0: Binary, 1: Binary_Inverted, 2: Threshold_Truncated, 3: Threshold_To_Zero, 4: Threshold_To_Zero_Inverted

Source
------

`src/nodelet/threshold_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/threshold_nodelet.cpp>`_, 
`launch/threshold.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/threshold.launch.xml>`_
