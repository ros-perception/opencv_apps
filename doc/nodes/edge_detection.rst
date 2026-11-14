edge_detection
==============

Detects the edges with the Sobel, Laplacian or Canny (``cv::Canny()``) edge detector.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Image filters, the nodes of which publish the processed image in ``~/image``

.. image:: https://ros-perception.github.io/opencv_apps/media/edge_detection.gif
   :alt: demo of edge_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps edge_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps edge_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps edge_detection.launch.xml image:=image edge_type:=2 node_name:=edge_canny_detection
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /edge_canny_detection/image

Canny.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps edge_detection.launch image:=camera/image_raw edge_type:=2 node_name:=edge_canny_detection

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
   * - ``L2gradient``
     - bool
     - false
     - yes
     - Flag, indicating whether a more accurate  L_2 norm should be used to calculate the image gradient magnitude ( L2gradient=true ), or whether the default  L_1 norm is enough ( L2gradient=false ).
   * - ``apertureSize``
     - int
     - 3 (1 - 10)
     - yes
     - Aperture size for the Sobel() operator.
   * - ``apply_blur_post``
     - bool
     - false
     - yes
     - Flag, applying GaussianBlur() to output(edge) image
   * - ``apply_blur_pre``
     - bool
     - true
     - yes
     - Flag, applying Blur() to input image
   * - ``canny_threshold1``
     - int
     - 100 (0 - 500)
     - yes
     - First threshold for the hysteresis procedure.
   * - ``canny_threshold2``
     - int
     - 200 (0 - 500)
     - yes
     - Second threshold for the hysteresis procedure.
   * - ``edge_type``
     - int
     - 0 (0 - 2)
     - yes
     - Edge Detection Methods, 0: Sobel, 1: Laplace, 2: Canny
   * - ``postBlurSigma``
     - double
     - 3.2 (0 - 10)
     - yes
     - Sigma for the GaussianBlur() operator.
   * - ``postBlurSize``
     - int
     - 13 (3 - 31)
     - yes
     - Aperture size for the Blur() operator.
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/edge_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/edge_detection_nodelet.cpp>`_, 
`launch/edge_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/edge_detection.launch.xml>`_
