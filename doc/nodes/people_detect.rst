people_detect
=============

Detects the people with the HOG descriptor and the linear SVM (``cv::HOGDescriptor``).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Detection, the nodes of which publish ``~/detections`` (DetectionArrayStamped)

.. image:: https://ros-perception.github.io/opencv_apps/media/people_detect.gif
   :alt: demo of people_detect

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps people_detect.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps people_detect.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_bridge.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_bridge.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps people_detect.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /people_detect/image

HOG needs full-body people of about 64x128 pixels, which this clip does not have, so most boxes are false.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher tos_bridge.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps people_detect.launch image:=camera/image_raw

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
   * - ``~/detections`` **(recommended)**
     - :ref:`opencv_apps/msg/DetectionArrayStamped <msg-DetectionArrayStamped>`
     - Recommended, opencv_apps 3.0.1 and later. The people, as the other detection nodes: the centers of the rectangles, the label ``person`` and the score of the HOG detector.
   * - ``~/found`` (compatibility)
     - :ref:`opencv_apps/msg/RectArrayStamped <msg-RectArrayStamped>`
     - Kept for compatibility, use ``~/detections``. The top-left corners of the rectangles.
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
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
   * - ``group_threshold``
     - int
     - 2 (0 - 100)
     - yes
     - Coefficient to regulate the similarity threshold. When detected, some objects can be covered by many rectangles. 0 means not to perform grouping.
   * - ``hit_threshold``
     - double
     - 0.0 (0 - 10)
     - yes
     - Threshold for the distance between features and SVM classifying plane.
   * - ``padding``
     - int
     - 32 (0 - 128)
     - yes
     - Padding of the detection window in pixels, of cv::HOGDescriptor::detectMultiScale().
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``scale0``
     - double
     - 1.05 (1 - 100)
     - yes
     - Coefficient of the detection window increase.
   * - ``win_stride``
     - int
     - 8 (1 - 100)
     - yes
     - Window stride. It must be a multiple of block stride.

Source
------

`src/nodelet/people_detect_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/people_detect_nodelet.cpp>`_, 
`launch/people_detect.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/people_detect.launch.xml>`_
