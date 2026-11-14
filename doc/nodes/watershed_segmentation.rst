watershed_segmentation
======================

Segments the image with the watershed algorithm (``cv::watershed()``) from the seed points, given by the mouse or the ``~/add_seed_points`` topic.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Contours, the nodes of which publish ``~/contours`` (ContourArrayStamped), ``~/hulls`` of convex_hull

.. image:: https://ros-perception.github.io/opencv_apps/media/watershed_segmentation.gif
   :alt: demo of watershed_segmentation

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps watershed_segmentation.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps watershed_segmentation.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/ballpit.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node ballpit.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps watershed_segmentation.launch.xml image:=image use_camera_info:=false
   # terminal 3: run, keep it running
   ros2 topic pub -r 1.5 /watershed_segmentation/add_seed_points opencv_apps/msg/Point2DArray "{points: [{x: 100.0, y: 100.0}, {x: 200.0, y: 100.0}, {x: 300.0, y: 100.0}, {x: 400.0, y: 100.0}, {x: 500.0, y: 100.0}, {x: 100.0, y: 200.0}, {x: 200.0, y: 200.0}, {x: 300.0, y: 200.0}, {x: 400.0, y: 200.0}, {x: 500.0, y: 200.0}, {x: 100.0, y: 300.0}, {x: 200.0, y: 300.0}, {x: 300.0, y: 300.0}, {x: 400.0, y: 300.0}, {x: 500.0, y: 300.0}]}"
   # terminal 4: view the result
   ros2 run rqt_image_view rqt_image_view /watershed_segmentation/image

See :doc:`../credits` for the license of the video.

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``image``
     - ``sensor_msgs/msg/Image``
   * - ``~/add_seed_points``
     - :ref:`opencv_apps/msg/Point2DArray <msg-Point2DArray>`

Published topics
----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``~/contours``
     - :ref:`opencv_apps/msg/ContourArrayStamped <msg-ContourArrayStamped>`
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
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/watershed_segmentation_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/watershed_segmentation_nodelet.cpp>`_, 
`launch/watershed_segmentation.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/watershed_segmentation.launch.xml>`_
