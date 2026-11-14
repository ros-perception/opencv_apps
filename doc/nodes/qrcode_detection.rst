qrcode_detection
================

Detects and decodes the QR codes (``cv::QRCodeDetector``).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Codes and markers, the nodes of which publish ``~/detections`` (DetectionArrayStamped), with the decoded data and the corners

.. image:: https://ros-perception.github.io/opencv_apps/media/qrcode_detection.gif
   :alt: demo of qrcode_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps qrcode_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps qrcode_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/codes.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node codes.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps qrcode_detection.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /qrcode_detection/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher codes.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps qrcode_detection.launch image:=camera/image_raw

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
     - Specify queue_size of input image subscribers

Source
------

`src/nodelet/qrcode_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/qrcode_detection_nodelet.cpp>`_, 
`launch/qrcode_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/qrcode_detection.launch.xml>`_
