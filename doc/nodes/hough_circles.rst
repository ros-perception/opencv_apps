hough_circles
=============

Detects the circles with the Hough transform (``cv::HoughCircles()``).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Features and shapes, the nodes of which publish ``~/corners`` (Point2DArrayStamped), ``~/lines`` (LineArrayStamped), ``~/circles`` (CircleArrayStamped) and ``~/contours`` (ContourArrayStamped, ``~/hulls`` of convex_hull), by the kind of the features

.. image:: https://ros-perception.github.io/opencv_apps/media/hough_circles.gif
   :alt: demo of hough_circles

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps hough_circles.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps hough_circles.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/pool_zoom.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node pool_zoom.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps hough_circles.launch.xml image:=image accumulator_threshold:=40.0 canny_threshold:=200.0 max_circle_radius:=60 min_circle_radius:=15
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /hough_circles/image

The radius range is tuned for the balls.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher pool_zoom.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps hough_circles.launch image:=camera/image_raw accumulator_threshold:=40.0 canny_threshold:=200.0 max_circle_radius:=60 min_circle_radius:=15

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
   * - ``~/circles``
     - :ref:`opencv_apps/msg/CircleArrayStamped <msg-CircleArrayStamped>`
   * - ``~/debug_image``
     - ``sensor_msgs/msg/Image``
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
   * - ``accumulator_threshold``
     - double
     - 50.0 (1 - 200)
     - yes
     - Accumulator threshold
   * - ``canny_threshold``
     - double
     - 200.0 (1 - 255)
     - yes
     - Canny threshold
   * - ``debug_image_type``
     - int
     - 0 (0 - 2)
     - yes
     - Select image type for debug output, 0: Input, 1: Blur, 2: Canny
   * - ``dp``
     - double
     - 2.0 (0 - 100)
     - yes
     - dp
   * - ``gaussian_blur_size``
     - int
     - 9 (1 - 30)
     - yes
     - the size of gaussian blur (should be odd number)
   * - ``gaussian_sigma_x``
     - double
     - 2.0 (1 - 10)
     - yes
     - sigma x of gaussian kernel
   * - ``gaussian_sigma_y``
     - double
     - 2.0 (1 - 10)
     - yes
     - sigma y of gaussian kernel
   * - ``max_circle_radius``
     - int
     - 0 (0 - 2000)
     - yes
     - the maximum size of the circle
   * - ``min_circle_radius``
     - int
     - 0 (0 - 500)
     - yes
     - the minimum size of the circle
   * - ``min_distance_between_circles``
     - double
     - 0.0 (0 - 1024)
     - yes
     - mnimum distance between the centers of the detected circles
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/hough_circles_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/hough_circles_nodelet.cpp>`_, 
`launch/hough_circles.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/hough_circles.launch.xml>`_
