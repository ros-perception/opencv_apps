face_detection
==============

Detects the faces and the eyes with the Haar cascade classifiers (``cv::CascadeClassifier``). See :doc:`dnn_face_detection` for the DNN based face detection.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.11 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Faces, the nodes of which publish ``~/faces`` (FaceArrayStamped)

.. image:: https://ros-perception.github.io/opencv_apps/media/face_detection.gif
   :alt: demo of face_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps face_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps face_detection.launch image:=/camera/image_raw debug_view:=false use_opencv4:=true

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_faces.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_faces.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps face_detection.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /face_detection/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher tos_faces.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps face_detection.launch image:=camera/image_raw use_opencv4:=true

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
   * - ``~/face_image``
     - ``sensor_msgs/msg/Image``
   * - ``~/faces``
     - :ref:`opencv_apps/msg/FaceArrayStamped <msg-FaceArrayStamped>`
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
   * - ``eyes_cascade_name``
     - string
     - "/usr/share/opencv4/haarcascades/haarcascade_eye_tree_eyeglasses.xml"
     - no, restart the node
     - Eye dtection cascade Filename
   * - ``face_cascade_name``
     - string
     - "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"
     - no, restart the node
     - Face dtection cascade Filename
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specigy queue_size of input image subscribers

Source
------

`src/nodelet/face_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/face_detection_nodelet.cpp>`_, 
`launch/face_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/face_detection.launch.xml>`_
