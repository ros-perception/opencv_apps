compare_histogram
=================

Computes the color histogram of the image (``cv::calcHist()``) and compares it with a reference histogram (``cv::compareHist()``), e.g. to find the scene or the object of the reference.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/compare_histogram.gif
   :alt: demo of compare_histogram

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps compare_histogram.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps compare_histogram.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   H=$(python3 -c "import cv2; img = cv2.VideoCapture('street.mp4').read()[1]; print([round(float(v), 6) for c in range(3) for v in cv2.normalize(cv2.calcHist([img], [c], None, [32], [0, 256]), None, 0, 1, cv2.NORM_MINMAX).flatten()])")  # the reference histogram of the first frame
   ros2 launch opencv_apps compare_histogram.launch.xml image:=image histogram_comparison_type:=3 histogram_size:=32 node_name:=compare_histogram_color reference_histogram:="$H"
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /compare_histogram_color/image

The reference histogram is that of the first frame of the clip, as test/ros2/test_compare_histogram.py.

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
   * - ``~/channel_distances``
     - ``std_msgs/msg/Float64MultiArray``
   * - ``~/distance``
     - ``std_msgs/msg/Float64``
   * - ``~/histogram``
     - ``std_msgs/msg/Float64MultiArray``
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
   * - ``accumulate``
     - bool
     - false
     - yes
     - Accumulation flag. If it is set, the histogram is not cleared between images
   * - ``histogram_comparison_type``
     - int
     - 0 (0 - 3)
     - yes
     - Histogram Comparison method, 0: Correlation, 1: Chi_Square, 2: Intersection, 3: Bhattacharyya
   * - ``histogram_size``
     - int
     - 256 (1 - 1024)
     - yes
     - Number of bins of the histogram of each channel
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``reference_histogram``
     - double[]
     - []
     - yes
     - Reference of the histogram comparison, channels x histogram_size values in the layout of ~/histogram

Source
------

`src/nodelet/compare_histogram_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/compare_histogram_nodelet.cpp>`_, 
`launch/compare_histogram.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/compare_histogram.launch.xml>`_
