hough_lines
===========

Detects the lines with the standard or the probabilistic Hough transform (``cv::HoughLines()``, ``cv::HoughLinesP()``).

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

.. image:: https://ros-perception.github.io/opencv_apps/media/hough_lines.gif
   :alt: demo of hough_lines

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps hough_lines.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps hough_lines.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps hough_lines.launch.xml image:=image hough_type:=1 node_name:=probabilistic_hough_lines threshold:=100
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /probabilistic_hough_lines/image

Probabilistic Hough transform.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps hough_lines.launch image:=camera/image_raw hough_type:=1 node_name:=probabilistic_hough_lines threshold:=100

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
   * - ``~/lines``
     - :ref:`opencv_apps/msg/LineArrayStamped <msg-LineArrayStamped>`

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
   * - ``hough_type``
     - int
     - 0 (0 - 1)
     - yes
     - Hough Line Methods, 0: Standard_Hough_Transform, 1: Probabilistic_Hough_Transform
   * - ``maxLineGap``
     - double
     - 10.0 (0 - 100)
     - yes
     - The maximum gap between two points to be considered in the same line.
   * - ``minLineLength``
     - double
     - 30.0 (0 - 500)
     - yes
     - The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``rho``
     - double
     - 1.0 (1 - 100)
     - yes
     - The resolution of the parameter r in pixels. We use 1 pixel.
   * - ``theta``
     - double
     - 1.0 (1 - 90)
     - yes
     - The resolution of the parameter theta in degrees.
   * - ``threshold``
     - int
     - 150 (50 - 150)
     - yes
     - Hough Line Threshold

Source
------

`src/nodelet/hough_lines_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/hough_lines_nodelet.cpp>`_, 
`launch/hough_lines.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/hough_lines.launch.xml>`_
