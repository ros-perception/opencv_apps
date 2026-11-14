camshift
========

Tracks a colored object with the CamShift algorithm (``cv::CamShift()``), by the back projection of the hue histogram of the region given by the mouse or the ``x``, ``y``, ``width`` and ``height`` parameters.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Tracking, the nodes of which publish ``~/output`` (RectArrayStamped), ``~/track_box`` (RotatedRectStamped) of camshift

.. image:: https://ros-perception.github.io/opencv_apps/media/camshift.gif
   :alt: demo of camshift

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps camshift.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps camshift.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/sign.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node sign.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps camshift.launch.xml image:=image height:=170 node_name:=camshift_roi width:=140 x:=410 y:=90
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /camshift_roi/image

The ROI is the face, and the ellipse also takes the red shirt of the same hue; back_project and image are shown.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher sign.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps camshift.launch image:=camera/image_raw height:=170 node_name:=camshift_roi width:=140 x:=410 y:=90

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
   * - ``~/back_project``
     - ``sensor_msgs/msg/Image``
   * - ``~/confidence``
     - ``std_msgs/msg/Float64``
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
   * - ``~/track_box``
     - :ref:`opencv_apps/msg/RotatedRectStamped <msg-RotatedRectStamped>`

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
   * - ``height``
     - int
     - 0 (0 - 4096)
     - yes
     - Height of the rectangle to track, 0 for none (setting it to 0 stops tracking)
   * - ``histogram``
     - double[]
     - []
     - no, restart the node
     - Histogram of tracked color object
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``smin``
     - int
     - 30 (1 - 256)
     - yes
     - Smin
   * - ``vmax``
     - int
     - 256 (1 - 256)
     - yes
     - Vmax
   * - ``vmin``
     - int
     - 10 (1 - 256)
     - yes
     - Vmin
   * - ``width``
     - int
     - 0 (0 - 4096)
     - yes
     - Width of the rectangle to track, 0 for none (setting it to 0 stops tracking)
   * - ``x``
     - int
     - 0 (0 - 4096)
     - yes
     - Center x of the rectangle to track
   * - ``y``
     - int
     - 0 (0 - 4096)
     - yes
     - Center y of the rectangle to track

Source
------

`src/nodelet/camshift_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/camshift_nodelet.cpp>`_, 
`launch/camshift.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/camshift.launch.xml>`_
