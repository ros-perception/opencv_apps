contour_moments
===============

Finds the contours and computes their moments (``cv::moments()``): the area, the length and the center of mass of each contour.

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

.. image:: https://ros-perception.github.io/opencv_apps/media/contour_moments.gif
   :alt: demo of contour_moments

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps contour_moments.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps contour_moments.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/pool_zoom.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node pool_zoom.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps contour_moments.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /contour_moments/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher pool_zoom.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps contour_moments.launch image:=camera/image_raw

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
     - Note
   * - ``~/contours`` **(recommended)**
     - :ref:`opencv_apps/msg/ContourArrayStamped <msg-ContourArrayStamped>`
     - Recommended, opencv_apps 3.0.1 and later. The contours, in the order of ``~/moments`` (by the area).
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
     -  
   * - ``~/moments``
     - :ref:`opencv_apps/msg/MomentArrayStamped <msg-MomentArrayStamped>`
     -  

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
   * - ``canny_low_threshold``
     - int
     - 100 (1 - 255)
     - yes
     - Canny Edge low Threshold
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/contour_moments_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/contour_moments_nodelet.cpp>`_, 
`launch/contour_moments.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/contour_moments.launch.xml>`_
