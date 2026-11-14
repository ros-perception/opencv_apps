dnn_image_classification
========================

Classify the images with a DNN model, e.g. PP-ResNet50 (ImageNet).

Image classification, by default with PP-ResNet50 (ImageNet 1000 classes) of opencv_zoo, or with a model of the same input (RGB, normalized by the mean and the standard deviation of ImageNet) and output (1 x classes), e.g. MobileNet of opencv_zoo. ~detections has the top_k classes in the order of the score, whose rect is the image.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Image classification, the nodes of which publish ``~/detections`` (DetectionArrayStamped), the classes with their scores

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_image_classification.gif
   :alt: demo of dnn_image_classification

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_image_classification.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_image_classification.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_bridge.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_bridge.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_image_classification.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_image_classification/image

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher tos_bridge.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_image_classification.launch image:=camera/image_raw

See :doc:`../credits` for the license of the video.

Models
------

The node runs the classification models of ImageNet with the same input (224x224, RGB normalized by the mean and the standard deviation of ImageNet) and output (the logits of the 1000 classes), given by ``model``. ImageNet has no class of the people, so the classes of this clip are the things around them.

PP-ResNet50 (default)
^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_image_classification.gif
   :alt: demo of dnn_image_classification with ppresnet50

.. code-block:: bash

   ros2 launch opencv_apps dnn_image_classification.launch.xml image:=image

MobileNetV1
^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_image_classification_mobilenetv1.gif
   :alt: demo of dnn_image_classification with mobilenetv1

.. code-block:: bash

   ros2 launch opencv_apps dnn_image_classification.launch.xml image:=image model:=https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/models/image_classification_mobilenet/image_classification_mobilenetv1_2022apr.onnx

MobileNetV1 of opencv_zoo, needs OpenCV 4.9 or later.

MobileNetV2
^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_image_classification_mobilenetv2.gif
   :alt: demo of dnn_image_classification with mobilenetv2

.. code-block:: bash

   ros2 launch opencv_apps dnn_image_classification.launch.xml image:=image model:=https://github.com/opencv/opencv_zoo/raw/48b10fa505aeb51fbd127fc60a5c660040e823b7/models/image_classification_mobilenet/image_classification_mobilenetv2_2022apr.onnx

MobileNetV2 of opencv_zoo, needs OpenCV 4.9 or later.

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
   * - ``classes``
     - string
     - ""
     - yes
     - Path or URL of the class names of the model, one class per line, or empty for the 1000 classes of ImageNet
   * - ``input_height``
     - int
     - 224 (1 - 4096)
     - yes
     - Height of the network input
   * - ``input_width``
     - int
     - 224 (1 - 4096)
     - yes
     - Width of the network input
   * - ``model``
     - string
     - ""
     - yes
     - Path or URL of the model (onnx), empty for PP-ResNet50 of opencv_zoo, e.g. MobileNet of opencv_zoo (OpenCV 4.9 or later). The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``softmax``
     - bool
     - true
     - yes
     - Apply softmax to the output of the model to get the probabilities, true for the models whose output is the logits (e.g. PP-ResNet50 and MobileNet of opencv_zoo), false for the models whose output is the probabilities
   * - ``top_k``
     - int
     - 5 (1 - 1000)
     - yes
     - Number of the classes to publish

Source
------

`src/nodelet/dnn_image_classification_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_image_classification_nodelet.cpp>`_, 
`launch/dnn_image_classification.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_image_classification.launch.xml>`_
