blob_detection
==============

Detects the blobs (regions of similar brightness or color) with ``cv::SimpleBlobDetector``, filtered by their area, circularity, convexity and inertia.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/blob_detection.gif
   :alt: demo of blob_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps blob_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps blob_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/ballpit.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node ballpit.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps blob_detection.launch.xml image:=hsv_color_filter/image min_area:=200.0
   # terminal 3: run, keep it running
   ros2 launch opencv_apps hsv_color_filter.launch.xml h_limit_min:=40 h_limit_max:=70 s_limit_min:=150 s_limit_max:=255 v_limit_min:=120 v_limit_max:=255
   # terminal 4: view the result
   ros2 run rqt_image_view rqt_image_view /blob_detection/image

The input is the mask of the yellow balls of hsv_color_filter.

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
   * - ``~/circles``
     - :ref:`opencv_apps/msg/CircleArrayStamped <msg-CircleArrayStamped>`
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
   * - ``blob_color``
     - int
     - 255 (0 - 255)
     - yes
     - 0 to extract dark blobs and 255 to extract light blobs
   * - ``filter_by_area``
     - bool
     - true
     - yes
     - Filter the blobs by the area
   * - ``filter_by_circularity``
     - bool
     - false
     - yes
     - Filter the blobs by the circularity (4*pi*area/perimeter^2)
   * - ``filter_by_color``
     - bool
     - true
     - yes
     - Filter the blobs by the color of the center
   * - ``filter_by_convexity``
     - bool
     - false
     - yes
     - Filter the blobs by the convexity (area / area of convex hull)
   * - ``filter_by_inertia``
     - bool
     - false
     - yes
     - Filter the blobs by the inertia ratio (the minimum / maximum second central moment, the square of minor axis / major axis for an ellipse, 0: line, 1: circle)
   * - ``max_area``
     - double
     - 100000.0 (0 - 1e+06)
     - yes
     - The maximum area (exclusive) of the blobs [pixel]
   * - ``max_circularity``
     - double
     - 1.0 (0 - 1)
     - yes
     - The maximum circularity (exclusive) of the blobs, 1.0 means no limit
   * - ``max_convexity``
     - double
     - 1.0 (0 - 1)
     - yes
     - The maximum convexity (exclusive) of the blobs, 1.0 means no limit
   * - ``max_inertia_ratio``
     - double
     - 1.0 (0 - 1)
     - yes
     - The maximum inertia ratio (exclusive) of the blobs, 1.0 means no limit
   * - ``max_threshold``
     - double
     - 220.0 (0 - 255)
     - yes
     - The maximum threshold (exclusive) to binarize the image
   * - ``min_area``
     - double
     - 25.0 (0 - 1e+06)
     - yes
     - The minimum area (inclusive) of the blobs [pixel]
   * - ``min_circularity``
     - double
     - 0.8 (0 - 1)
     - yes
     - The minimum circularity (inclusive) of the blobs
   * - ``min_convexity``
     - double
     - 0.95 (0 - 1)
     - yes
     - The minimum convexity (inclusive) of the blobs
   * - ``min_dist_between_blobs``
     - double
     - 10.0 (0 - 1000)
     - yes
     - The blobs located closer than min_dist_between_blobs are merged
   * - ``min_inertia_ratio``
     - double
     - 0.1 (0 - 1)
     - yes
     - The minimum inertia ratio (inclusive) of the blobs
   * - ``min_repeatability``
     - int
     - 2 (1 - 255)
     - yes
     - The minimum number of the binarized images in which a blob is detected
   * - ``min_threshold``
     - double
     - 50.0 (0 - 255)
     - yes
     - The minimum threshold to binarize the image
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``threshold_step``
     - double
     - 10.0 (1 - 255)
     - yes
     - Step of the thresholds from min_threshold to max_threshold

Source
------

`src/nodelet/blob_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/blob_detection_nodelet.cpp>`_, 
`launch/blob_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/blob_detection.launch.xml>`_
