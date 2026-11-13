dnn_facial_expression_recognition
=================================

Recognize the facial expressions with YuNet and MobileFaceNet.

Facial expression recognition: the faces are detected by YuNet (cv::FaceDetectorYN), aligned by their 5 landmarks, and their expressions (angry, disgust, fearful, happy, neutral, sad or surprised) are recognized by MobileFaceNet of opencv_zoo. ~faces has the expression in label and its probability in confidence of each face.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_facial_expression_recognition.gif
   :alt: demo of dnn_facial_expression_recognition

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_facial_expression_recognition.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_facial_expression_recognition.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_faces.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_faces.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_facial_expression_recognition.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_facial_expression_recognition/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher tos_faces.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_facial_expression_recognition.launch image:=camera/image_raw

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
   * - ``detection_model``
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
   * - ``recognition_model``
     - string
     - ""
     - yes
     - Path or URL of the facial expression recognition model (onnx), empty for MobileFaceNet of opencv_zoo
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

`src/nodelet/dnn_facial_expression_recognition_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_facial_expression_recognition_nodelet.cpp>`_, 
`launch/dnn_facial_expression_recognition.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_facial_expression_recognition.launch.xml>`_
