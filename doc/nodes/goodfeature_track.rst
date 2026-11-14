goodfeature_track
=================

Detects the strong corners to track with the Shi-Tomasi method (``cv::goodFeaturesToTrack()``), and publishes their positions.

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

.. image:: https://ros-perception.github.io/opencv_apps/media/goodfeature_track.gif
   :alt: demo of goodfeature_track

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps goodfeature_track.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps goodfeature_track.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps goodfeature_track.launch.xml image:=image max_corners:=50
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /goodfeature_track/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps goodfeature_track.launch image:=camera/image_raw max_corners:=50

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
   * - ``~/corners``
     - :ref:`opencv_apps/msg/Point2DArrayStamped <msg-Point2DArrayStamped>`
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
   * - ``max_corners``
     - int
     - 23 (1 - 100)
     - yes
     - Max Number of Corners
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/goodfeature_track_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/goodfeature_track_nodelet.cpp>`_, 
`launch/goodfeature_track.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/goodfeature_track.launch.xml>`_
