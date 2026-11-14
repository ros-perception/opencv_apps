aruco_detection
===============

Detects the ArUco markers (``cv::aruco``), and their poses with the camera info and the size of the markers.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.7 or later, or the aruco module of opencv_contrib
   * - Group
     - Detection, the nodes of which publish ``~/detections`` (DetectionArrayStamped)

.. image:: https://ros-perception.github.io/opencv_apps/media/aruco_detection.gif
   :alt: demo of aruco_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps aruco_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps aruco_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/codes.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node codes.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps aruco_detection.launch.xml image:=image marker_length:=0.05 use_camera_info:=true
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /aruco_detection/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher codes.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps aruco_detection.launch image:=camera/image_raw marker_length:=0.05 use_camera_info:=true

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
   * - ``~/detections``
     - :ref:`opencv_apps/msg/DetectionArrayStamped <msg-DetectionArrayStamped>`
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
   * - ``~/poses``
     - ``geometry_msgs/msg/PoseArray``

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
   * - ``dictionary``
     - string
     - "DICT_4X4_50"
     - yes
     - Predefined dictionary of the markers, e.g. DICT_4X4_50, DICT_6X6_250, DICT_ARUCO_ORIGINAL, DICT_APRILTAG_36h11
   * - ``marker_length``
     - double
     - 0.0 (0 - 100)
     - yes
     - Length of the side of the markers in meters, to publish their poses with the camera info, 0 not to publish
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``rectified``
     - bool
     - false
     - yes
     - The image is rectified (e.g. image_rect of image_proc), then the poses are estimated with the intrinsics of P of the camera info without distortion, instead of K and D
   * - ``use_camera_info``
     - bool
     - false
     - yes
     - Indicates that the camera_info topic should be subscribed to, to get the camera model for the poses of the markers. Otherwise the poses are not published.

Source
------

`src/nodelet/aruco_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/aruco_detection_nodelet.cpp>`_, 
`launch/aruco_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/aruco_detection.launch.xml>`_
