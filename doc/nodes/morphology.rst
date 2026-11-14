morphology
==========

Applies a morphological transformation (``cv::morphologyEx()``): opening, closing, gradient, top hat or black hat.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 2.0.3 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Image filters, the nodes of which publish the processed image in ``~/image``

.. image:: https://ros-perception.github.io/opencv_apps/media/morphology.gif
   :alt: demo of morphology

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps morphology.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps morphology.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps morphology.launch.xml image:=image morph_element:=0 morph_operator:=0 morph_size:=10
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /morphology/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps morphology.launch image:=camera/image_raw morph_element:=0 morph_operator:=0 morph_size:=10

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
   * - ``morph_element``
     - int
     - 0 (0 - 2)
     - yes
     - Type of the kernel 0: Rect, 1: Cross, 2: Ellipse
   * - ``morph_operator``
     - int
     - 1 (0 - 4)
     - yes
     - Morphology Operation Methods, 0: Opening, 1: Closing, 2: Gradient, 3: Top_Hat, 4: Black_Hat
   * - ``morph_size``
     - int
     - 0 (0 - 101)
     - yes
     - Radius of the kernel, whose size is 2 * morph_size + 1.
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/morphology_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/morphology_nodelet.cpp>`_, 
`launch/morphology.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/morphology.launch.xml>`_
