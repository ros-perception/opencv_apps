match_template
==============

Finds the template image in the image (``cv::matchTemplate()``), and publishes the matched regions. The template is given by the ``~/template`` topic, a file, the mouse or the ``~/set_roi`` service.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 3.0 or later

.. image:: https://ros-perception.github.io/opencv_apps/media/match_template.gif
   :alt: demo of match_template

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps match_template.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps match_template.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_canal.mp4
   ffmpeg -i tos_canal.mp4 -frames:v 1 frame0.png  # the first frame
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_canal.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps match_template.launch.xml image:=image
   # terminal 3: then run once
   ros2 run opencv_apps set_image.py 305 90 70 90 frame0.png --ros-args -r set_roi:=/match_template/set_roi
   # terminal 4: view the result
   ros2 run rqt_image_view rqt_image_view /match_template/image

Set the face of the man on the first frame with set_image.py just after the video starts (the demo publishes the frames one by one and sets it at the start of each loop).

See :doc:`../credits` for the license of the video.

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``image``
     - ``sensor_msgs/msg/Image``
   * - ``~/mask``
     - ``sensor_msgs/msg/Image``
   * - ``~/template``
     - ``sensor_msgs/msg/Image``

Published topics
----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``~/image``
     - ``sensor_msgs/msg/Image``
   * - ``~/matched_image``
     - ``sensor_msgs/msg/Image``
   * - ``~/output``
     - :ref:`opencv_apps/msg/RectArrayStamped <msg-RectArrayStamped>`
   * - ``~/score``
     - ``std_msgs/msg/Float64``

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
   * - ``mask_file``
     - string
     - ""
     - yes
     - File name of the mask image of the template, non-zero pixels are used
   * - ``match_method``
     - int
     - 5 (0 - 5)
     - yes
     - Matching Method, 0: TM_SQDIFF, 1: TM_SQDIFF_NORMED, 2: TM_CCORR, 3: TM_CCORR_NORMED, 4: TM_CCOEFF, 5: TM_CCOEFF_NORMED
   * - ``match_threshold``
     - double
     - 0.0 (0 - 1)
     - yes
     - Minimum similarity of a match, the score of TM_CCORR_NORMED and TM_CCOEFF_NORMED, 1 - score of TM_SQDIFF_NORMED; not used by the methods which are not normalized
   * - ``max_matches``
     - int
     - 1 (1 - 100)
     - yes
     - Maximum number of the matches. 1: the best match only, more than 1: the matches above match_threshold, after a non-maximum suppression
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``template_file``
     - string
     - ""
     - yes
     - File name of the template image

Source
------

`src/nodelet/match_template_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/match_template_nodelet.cpp>`_, 
`launch/match_template.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/match_template.launch.xml>`_
