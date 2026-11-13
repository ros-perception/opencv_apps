general_contours
================

Finds the contours of the thresholded image, and their bounding rotated rectangles and ellipses (``cv::minAreaRect()``, ``cv::fitEllipse()``).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/general_contours.gif
   :alt: demo of general_contours

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps general_contours.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps general_contours.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/pool_zoom.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node pool_zoom.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps general_contours.launch.xml image:=image threshold:=30
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /general_contours/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher pool_zoom.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps general_contours.launch image:=camera/image_raw threshold:=30

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
   * - ``~/ellipses``
     - :ref:`opencv_apps/msg/RotatedRectArrayStamped <msg-RotatedRectArrayStamped>`
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
   * - ``~/rectangles``
     - :ref:`opencv_apps/msg/RotatedRectArrayStamped <msg-RotatedRectArrayStamped>`

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
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``threshold``
     - int
     - 100 (1 - 255)
     - yes
     - Detect edges using Threshold

Source
------

`src/nodelet/general_contours_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/general_contours_nodelet.cpp>`_, 
`launch/general_contours.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/general_contours.launch.xml>`_
