dnn_person_reid
===============

Re-identify the people detected by another node with the YouTu person ReID model.

Person re-identification of the people detected by another node (e.g. ~/detections of dnn_object_detection, synchronized with the image) with the YouTu ReID baseline of opencv_zoo (Apache 2.0): the feature of each person is compared with the gallery of the features of the people seen before by the cosine similarity, and the person gets the id of the most similar one over match_threshold, or a new id. Each person of the gallery is matched with one person of an image, the most similar pairs first, and its feature is updated by the moving average. The detections are published with the id of the person in their data (in decimal), with their class id kept in their id.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.1.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - OpenCV 4.5.4 or later
   * - Group
     - Re-identification, the nodes of which publish ``~/detections`` (DetectionArrayStamped), with the id of each person in ``data``

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_person_reid.gif
   :alt: demo of dnn_person_reid

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_person_reid.launch.xml image:=/camera/image_raw input:=/dnn_object_detection/detections debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_person_reid.launch image:=/camera/image_raw input:=/dnn_object_detection/detections debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tum_walking.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tum_walking.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_person_reid.launch.xml image:=image input:=/dnn_object_detection/detections
   # terminal 3: run, keep it running
   ros2 launch opencv_apps dnn_object_detection.launch.xml image:=image debug_view:=false
   # terminal 4: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_person_reid/image

The people of dnn_object_detection keep their ids while they walk around: each box is in the color of the id, with person_<id> and the cosine similarity to the person seen before, or new. The YouTu ReID baseline of opencv_zoo (Apache 2.0, 106 MB, onnx) is read by OpenCV 4 (also 4.6 of Ubuntu 24.04) and 5.

See :doc:`../credits` for the license of the video.

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``image``
     - ``sensor_msgs/msg/Image``
   * - ``input``
     - :ref:`opencv_apps/msg/DetectionArrayStamped <msg-DetectionArrayStamped>`

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
   * - ``approximate_sync``
     - bool
     - false
     - no, restart the node
     - Synchronize the image and the detections by the approximate time of the stamps, instead of the exact time
   * - ``backend``
     - string
     - "auto"
     - yes
     - Backend of cv::dnn: auto, cpu, opencl, opencl_fp16, cuda or cuda_fp16. auto uses CUDA when OpenCV is built with CUDA and a CUDA device is found, and the CPU otherwise.
   * - ``label``
     - string
     - "person"
     - yes
     - Label of the detections to re-identify, empty for all of them
   * - ``match_threshold``
     - double
     - 0.5 (-1 - 1)
     - yes
     - Threshold of the cosine similarity to the person seen before to have the same id, a new id otherwise
   * - ``model``
     - string
     - ""
     - yes
     - Path or URL of the person ReID model (onnx, 128x256 RGB input), empty for the YouTu ReID model of opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``queue_size``
     - int
     - 10
     - no, restart the node
     - Queue size of the synchronized subscribers of the image and the detections.
   * - ``sync_queue_size``
     - int
     - 100
     - no, restart the node
     - Number of the images (and the detections) kept to be synchronized, more than the rate of the images times the latency of the detector, e.g. 30 Hz x 0.6 s = 18
   * - ``update_rate``
     - double
     - 0.1 (0 - 1)
     - yes
     - Rate of the feature of the new image in the moving average of the feature of the person

Source
------

`src/nodelet/dnn_person_reid_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_person_reid_nodelet.cpp>`_, 
`launch/dnn_person_reid.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_person_reid.launch.xml>`_
