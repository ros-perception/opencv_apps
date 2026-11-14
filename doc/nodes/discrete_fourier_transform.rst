discrete_fourier_transform
==========================

Shows the magnitude spectrum of the discrete Fourier transform (``cv::dft()``) of the image.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11.15 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/discrete_fourier_transform.gif
   :alt: demo of discrete_fourier_transform

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps discrete_fourier_transform.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps discrete_fourier_transform.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps discrete_fourier_transform.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /discrete_fourier_transform/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps discrete_fourier_transform.launch image:=camera/image_raw

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
   * - ``queue_size``
     - int
     - 1
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/discrete_fourier_transform_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/discrete_fourier_transform_nodelet.cpp>`_, 
`launch/discrete_fourier_transform.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/discrete_fourier_transform.launch.xml>`_
