segment_objects
===============

Segments the moving objects by the background subtraction, and cleans up the foreground with the connected components, publishing their contours and area.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Segmentation, the nodes of which publish the mask or label image, ``~/image`` of the color filters and background_subtraction, ``~/label`` of dnn_semantic_segmentation, and ``~/contours`` (ContourArrayStamped) of the regions of segment_objects and watershed_segmentation

.. image:: https://ros-perception.github.io/opencv_apps/media/segment_objects.gif
   :alt: demo of segment_objects

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps segment_objects.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps segment_objects.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/pool.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node pool.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps segment_objects.launch.xml image:=image use_camera_info:=false
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /segment_objects/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher pool.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps segment_objects.launch image:=camera/image_raw use_camera_info:=false

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
   * - ``~/area``
     - ``std_msgs/msg/Float64``
   * - ``~/contours``
     - :ref:`opencv_apps/msg/ContourArrayStamped <msg-ContourArrayStamped>`
   * - ``~/image``
     - ``sensor_msgs/msg/Image``

Services
--------

.. list-table::
   :header-rows: 1

   * - Service
     - Type
   * - ``~/update_bg_model``
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
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/segment_objects_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/segment_objects_nodelet.cpp>`_, 
`launch/segment_objects.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/segment_objects.launch.xml>`_
