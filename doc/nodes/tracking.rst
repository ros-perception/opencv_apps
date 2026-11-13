tracking
========

Tracks the target with the trackers of OpenCV: MIL, BOOSTING, MEDIANFLOW, TLD, KCF, MOSSE and CSRT. The target is given by the mouse or the ``~/set_roi`` service, with the same interface as :doc:`dnn_tracking` and :doc:`match_template`.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later; the trackers other than MIL need libopencv-contrib
   * - OpenCV
     - OpenCV 4.5.1 or later, and the tracking module of opencv_contrib for the trackers other than MIL

.. image:: https://ros-perception.github.io/opencv_apps/media/tracking.gif
   :alt: demo of tracking

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps tracking.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps tracking.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_canal.mp4
   ffmpeg -i tos_canal.mp4 -frames:v 1 frame0.png  # the first frame
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_canal.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps tracking.launch.xml image:=image tracking_algorithm:=7
   # terminal 3: then run once
   ros2 run opencv_apps set_image.py 300 160 160 220 frame0.png --ros-args -r set_roi:=/tracking/set_roi
   # terminal 4: view the result
   ros2 run rqt_image_view rqt_image_view /tracking/image

Set the man on the first frame with set_image.py just after the video starts (the demo publishes the frames one by one and sets it at the start of each loop).

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
   * - ``~/output``
     - :ref:`opencv_apps/msg/RectArrayStamped <msg-RectArrayStamped>`

Services
--------

.. list-table::
   :header-rows: 1

   * - Service
     - Type
   * - ``~/set_roi``
     - ``opencv_apps/srv/SetROI``

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
   * - ``tracking_algorithm``
     - int
     - 4 (0 - 7)
     - yes
     - Tracking Algorithm, 0: MIL, 1: BOOSTING, 2: MEDIANFLOW, 3: TLD, 4: KCF, 6: MOSSE, 7: CSRT

Source
------

`src/nodelet/tracking_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/tracking_nodelet.cpp>`_, 
`launch/tracking.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/tracking.launch.xml>`_
