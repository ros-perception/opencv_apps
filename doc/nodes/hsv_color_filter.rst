hsv_color_filter
================

Extracts the pixels in a range of hue, saturation and value. ``~/color_space`` shows the colors of the image and the range in the HSV color space as a point cloud.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11.15 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Color and region masks, the nodes of which publish the mask or label image, ``~/image`` of the color filters and background_subtraction, ``~/label`` of dnn_semantic_segmentation

.. image:: https://ros-perception.github.io/opencv_apps/media/hsv_color_filter.gif
   :alt: demo of hsv_color_filter

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps hsv_color_filter.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps hsv_color_filter.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/ballpit.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node ballpit.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps hsv_color_filter.launch.xml image:=image h_limit_max:=70 h_limit_min:=40 s_limit_max:=255 s_limit_min:=150 v_limit_max:=255 v_limit_min:=120
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /hsv_color_filter/image

The yellow balls.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher ballpit.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps hsv_color_filter.launch image:=camera/image_raw h_limit_max:=70 h_limit_min:=40 s_limit_max:=255 s_limit_min:=150 v_limit_max:=255 v_limit_min:=120

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
   * - ``~/color_space``
     - ``sensor_msgs/msg/PointCloud2``
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
   * - ``color_space_step``
     - int
     - 4 (1 - 64)
     - no, restart the node
     - Publish every color_space_step-th pixel in x and y of the image to ~color_space
   * - ``h_limit_max``
     - int
     - 360 (0 - 360)
     - yes
     - The maximum allowed field value Hue
   * - ``h_limit_min``
     - int
     - 0 (0 - 360)
     - yes
     - The minimum allowed field value Hue
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``s_limit_max``
     - int
     - 256 (0 - 256)
     - yes
     - The maximum allowed field value Saturation
   * - ``s_limit_min``
     - int
     - 0 (0 - 256)
     - yes
     - The minimum allowed field value Saturation
   * - ``v_limit_max``
     - int
     - 256 (0 - 256)
     - yes
     - The maximum allowed field value Value
   * - ``v_limit_min``
     - int
     - 0 (0 - 256)
     - yes
     - The minimum allowed field value Value

Source
------

`src/nodelet/color_filter_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/color_filter_nodelet.cpp>`_, 
`launch/hsv_color_filter.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/hsv_color_filter.launch.xml>`_
