dnn_face_detection
==================

Detect faces with YuNet (cv::FaceDetectorYN).

Face detection with YuNet (cv::FaceDetectorYN), https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Faces, the nodes of which publish ``~/faces`` (FaceArrayStamped)

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_face_detection.gif
   :alt: demo of dnn_face_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_face_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_face_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_faces.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_faces.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_face_detection.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_face_detection/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher tos_faces.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_face_detection.launch image:=camera/image_raw

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
   * - ``backend``
     - string
     - "auto"
     - yes
     - Backend of cv::dnn: auto, cpu, opencl, opencl_fp16, cuda or cuda_fp16. auto uses CUDA when OpenCV is built with CUDA and a CUDA device is found, and the CPU otherwise.
   * - ``model``
     - string
     - ""
     - yes
     - Path or URL of the YuNet model (onnx), empty for the default for this OpenCV version. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``nms_threshold``
     - double
     - 0.3 (0 - 1)
     - yes
     - Threshold of the overlap of the faces to suppress, by non maximum suppression
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``score_threshold``
     - double
     - 0.9 (0 - 1)
     - yes
     - Threshold of the score to detect the faces
   * - ``top_k``
     - int
     - 5000 (1 - 100000)
     - yes
     - Number of the candidates to keep before non maximum suppression

Source
------

`src/nodelet/dnn_face_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_face_detection_nodelet.cpp>`_, 
`launch/dnn_face_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_face_detection.launch.xml>`_
