lk_flow
=======

Tracks the feature points with the sparse Lucas-Kanade optical flow (``cv::calcOpticalFlowPyrLK()``). The points are given by the services, or by the corner detection.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Optical flow and motion, the nodes of which publish ``~/flows`` (FlowArrayStamped), ``~/shift`` (Point2DStamped) of phase_corr

.. image:: https://ros-perception.github.io/opencv_apps/media/lk_flow.gif
   :alt: demo of lk_flow

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps lk_flow.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps lk_flow.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/keybridge.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node keybridge.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps lk_flow.launch.xml image:=image_rect_throttle use_camera_info:=false
   # terminal 3: run, keep it running
   ros2 run topic_tools throttle messages image 5 image_rect_throttle
   # terminal 4: then run once
   ros2 service call /lk_flow/initialize_points std_srvs/srv/Empty
   # terminal 5: view the result
   ros2 run rqt_image_view rqt_image_view /lk_flow/image

The input is throttled to 5 Hz, and the points to track are initialized by the service.

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
   * - ``~/flows``
     - :ref:`opencv_apps/msg/FlowArrayStamped <msg-FlowArrayStamped>`
   * - ``~/image``
     - ``sensor_msgs/msg/Image``

Services
--------

.. list-table::
   :header-rows: 1

   * - Service
     - Type
   * - ``~/delete_points``
     - ``std_srvs/srv/Empty``
   * - ``~/initialize_points``
     - ``std_srvs/srv/Empty``
   * - ``~/toggle_night_mode``
     - ``std_srvs/srv/Empty``

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
   * - ``block_size``
     - int
     - 3 (1 - 100)
     - yes
     - Size of an average block for computing a derivative covariation matrix over each pixel neighborhood.
   * - ``harris_k``
     - double
     - 0.04 (0.001 - 10)
     - yes
     - Free parameter of the Harris detector
   * - ``min_distance``
     - int
     - 10 (1 - 100)
     - yes
     - Minimum possible Euclidean distance between the returned corners.
   * - ``quality_level``
     - double
     - 0.01 (0.001 - 10)
     - yes
     - Parameter characterizing the minimal acceptance quality of image corners.
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/lk_flow_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/lk_flow_nodelet.cpp>`_, 
`launch/lk_flow.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/lk_flow.launch.xml>`_
