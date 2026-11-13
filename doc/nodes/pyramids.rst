pyramids
========

Down-samples or up-samples the image with the Gaussian image pyramid (``cv::pyrDown()``, ``cv::pyrUp()``).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 2.0.0 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions

.. image:: https://ros-perception.github.io/opencv_apps/media/pyramids.gif
   :alt: demo of pyramids

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps pyramids.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps pyramids.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/street.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node street.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps pyramids.launch.xml image:=image node_name:=pyramids_zoomout_2 num_of_pyramids:=2 pyramids_type:=1
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /pyramids_zoomout_2/image

Zoom out twice.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher street.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps pyramids.launch image:=camera/image_raw node_name:=pyramids_zoomout_2 num_of_pyramids:=2 pyramids_type:=1

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
   * - ``num_of_pyramids``
     - int
     - 1 (1 - 10)
     - yes
     - Number of pyramids
   * - ``pyramids_type``
     - int
     - 0 (0 - 2)
     - yes
     - Pyramids Type, 0: Up, 1: Down
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/pyramids_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/pyramids_nodelet.cpp>`_, 
`launch/pyramids.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/pyramids.launch.xml>`_
