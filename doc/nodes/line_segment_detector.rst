line_segment_detector
=====================

Detects the line segments with the line segment detector (LSD, ``cv::LineSegmentDetector``).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 3.0 - 3.4.5 or 4.5.4 or later, which have cv::createLineSegmentDetector()

.. image:: https://ros-perception.github.io/opencv_apps/media/line_segment_detector.gif
   :alt: demo of line_segment_detector

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps line_segment_detector.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps line_segment_detector.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps line_segment_detector.launch.xml image:=image lsd_density_threshold:=0.3 lsd_line_length_threshold:=10.0 lsd_refine_type:=2 lsd_scale:=0.4
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /line_segment_detector/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps line_segment_detector.launch image:=camera/image_raw lsd_density_threshold:=0.3 lsd_line_length_threshold:=10.0 lsd_refine_type:=2 lsd_scale:=0.4

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
   * - ``~/lines``
     - :ref:`opencv_apps/msg/LineArrayStamped <msg-LineArrayStamped>`

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
   * - ``lsd_angle_threshold``
     - double
     - 22.5 (0.1 - 179)
     - yes
     - Gradient angle tolerance in degrees
   * - ``lsd_density_threshold``
     - double
     - 0.7 (0 - 0.9)
     - yes
     - Minimal density of aligned region points in the enclosing rectangle
   * - ``lsd_line_length_threshold``
     - double
     - 100.0 (0 - 1000)
     - yes
     - Threshold of line length. Shorter lines are not published nor drawn.
   * - ``lsd_log_eps``
     - double
     - 0.0 (0 - 360)
     - yes
     - Detection threshold: -log10(NFA) > log_eps. Used only when advanced refinement is chosen
   * - ``lsd_n_bins``
     - int
     - 1024 (1 - 10000)
     - yes
     - Number of bins in pseudo-ordering of gradient modulus
   * - ``lsd_quant``
     - double
     - 2.0 (0 - 100)
     - yes
     - Bound to the quantization error on the gradient norm
   * - ``lsd_refine_type``
     - int
     - 0 (0 - 2)
     - yes
     - Line Segment Detector Modes, 0: REFINE_NONE, 1: REFINE_STD, 2: REFINE_ADV
   * - ``lsd_scale``
     - double
     - 0.8 (0.1 - 1)
     - yes
     - The scale of the image that will be used to find the lines. Range [0.1..1]
   * - ``lsd_sigma_scale``
     - double
     - 0.6 (0.1 - 10)
     - yes
     - Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers

Source
------

`src/nodelet/line_segment_detector_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/line_segment_detector_nodelet.cpp>`_, 
`launch/line_segment_detector.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/line_segment_detector.launch.xml>`_
