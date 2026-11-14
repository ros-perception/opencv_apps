equalize_histogram
==================

Equalizes the histogram of the image (``cv::equalizeHist()`` or CLAHE) to improve its contrast.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 2.0.3 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/equalize_histogram.gif
   :alt: demo of equalize_histogram

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps equalize_histogram.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps equalize_histogram.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps equalize_histogram.launch.xml image:=image clahe_clip_limit:=20.0 clahe_tile_size_x:=32 clahe_tile_size_y:=32 histogram_equalization_type:=1 node_name:=equalize_histogram_clahe use_opencl:=false
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /equalize_histogram_clahe/image

CLAHE.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps equalize_histogram.launch image:=camera/image_raw clahe_clip_limit:=20.0 clahe_tile_size_x:=32 clahe_tile_size_y:=32 histogram_equalization_type:=1 node_name:=equalize_histogram_clahe use_opencl:=false

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
   * - ``clahe_clip_limit``
     - double
     - 20.0 (0 - 255)
     - yes
     - Clip Limit for Clahe filter
   * - ``clahe_tile_size_x``
     - int
     - 32 (0 - 300)
     - yes
     - X Tile Size for Clahe filter
   * - ``clahe_tile_size_y``
     - int
     - 32 (0 - 300)
     - yes
     - Y Tile Size for Clahe filter
   * - ``histogram_equalization_type``
     - int
     - 0 (0 - 1)
     - yes
     - Histogram Equalization Methods, 0: EqualizeHist, 1: Clahe
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``use_opencl``
     - bool
     - true
     - no, restart the node
     - Enable or disable OpenCL

Source
------

`src/nodelet/equalize_histogram_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/equalize_histogram_nodelet.cpp>`_, 
`launch/equalize_histogram.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/equalize_histogram.launch.xml>`_
