dnn_text_detection
==================

Detect and recognize texts with DB (cv::dnn::TextDetectionModel_DB) and CRNN (cv::dnn::TextRecognitionModel).

Text detection and recognition (OCR) with DB (cv::dnn::TextDetectionModel_DB) and CRNN (cv::dnn::TextRecognitionModel), by default DB (ResNet-18, trained on IC15) and CRNN (English) of opencv_zoo.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Text detection and recognition, the nodes of which publish ``~/detections`` (DetectionArrayStamped), with the recognized text

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_text_detection.gif
   :alt: demo of dnn_text_detection

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_text_detection.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_text_detection.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/text.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node text.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_text_detection.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_text_detection/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher text.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_text_detection.launch image:=camera/image_raw

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
   * - ``~/detections``
     - :ref:`opencv_apps/msg/DetectionArrayStamped <msg-DetectionArrayStamped>`
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
   * - ``binary_threshold``
     - double
     - 0.3 (0 - 1)
     - yes
     - Threshold of the text probability of the pixels
   * - ``detection_model``
     - string
     - ""
     - yes
     - Path or URL of the text detection model of DB (onnx), empty for DB (ResNet-18, IC15) of opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``input_height``
     - int
     - 736 (32 - 4096)
     - yes
     - Height of the input of the detection model, a multiple of 32
   * - ``input_width``
     - int
     - 736 (32 - 4096)
     - yes
     - Width of the input of the detection model, a multiple of 32
   * - ``max_candidates``
     - int
     - 200 (1 - 10000)
     - yes
     - Maximum number of the text regions
   * - ``polygon_threshold``
     - double
     - 0.5 (0 - 1)
     - yes
     - Threshold of the score of the text regions
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``recognition_model``
     - string
     - ""
     - yes
     - Path or URL of the text recognition model of CRNN (onnx), empty for CRNN (English) of opencv_zoo, or none to detect the text only
   * - ``unclip_ratio``
     - double
     - 2.0 (0 - 10)
     - yes
     - Ratio to expand the detected text regions
   * - ``vocabulary``
     - string
     - ""
     - yes
     - Path or URL of the vocabulary of the recognition model, one character per line, or empty for 0-9 and a-z of CRNN (English)

Source
------

`src/nodelet/dnn_text_detection_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_text_detection_nodelet.cpp>`_, 
`launch/dnn_text_detection.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_text_detection.launch.xml>`_
