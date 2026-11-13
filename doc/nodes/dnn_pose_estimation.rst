dnn_pose_estimation
===================

Estimate the human poses with MediaPipe person detection and pose estimation.

Human pose estimation with MediaPipe BlazePose of opencv_zoo (Apache 2.0): the persons are detected by the person detection model, then the 33 landmarks of each person are estimated by the pose estimation model, on the image rotated so that the person stands upright. As MediaPipe, the persons are tracked by the auxiliary landmarks of their poses in the previous image, and the person detection runs every detection_interval images to find the new persons, or when no person is tracked. The landmarks of the tracked persons are smoothed by One Euro filter, as MediaPipe. The models need OpenCV 4.9 or later (OpenCV 4.6 cannot read them). ~skeletons has a Skeleton for each person: label "person", id the id while tracked, score the confidence of the pose, rect the bounding box of the landmarks, and keypoints the 33 landmarks in the image coordinates with their visibility and presence, in the order of MediaPipe (LANDMARK_NAMES).

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - lyrical and later; not ROS One (OpenCV 4.6), jazzy (OpenCV 4.6), kilted (OpenCV 4.6)
   * - OpenCV
     - OpenCV 4.9 or later, which reads the MediaPipe models

.. image:: https://ros-perception.github.io/opencv_apps/media/dnn_pose_estimation.gif
   :alt: demo of dnn_pose_estimation

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps dnn_pose_estimation.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps dnn_pose_estimation.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/tos_bridge.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node tos_bridge.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node
   ros2 launch opencv_apps dnn_pose_estimation.launch.xml image:=image
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /dnn_pose_estimation/image

Needs OpenCV 4.9 or later; recorded with OpenCV 4.12 on CUDA.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher tos_bridge.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps dnn_pose_estimation.launch image:=camera/image_raw

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
   * - ``~/skeletons``
     - :ref:`opencv_apps/msg/SkeletonArrayStamped <msg-SkeletonArrayStamped>`

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
   * - ``confidence_threshold``
     - double
     - 0.5 (0 - 1)
     - yes
     - Threshold of the confidence to detect the persons and to estimate their poses
   * - ``detection_interval``
     - int
     - 10 (1 - 1000)
     - yes
     - Interval of the person detection to find the new persons while tracking
   * - ``nms_threshold``
     - double
     - 0.3 (0 - 1)
     - yes
     - Threshold of the overlap of the persons to suppress, by non maximum suppression
   * - ``person_detection_model``
     - string
     - ""
     - yes
     - Path or URL of the MediaPipe person detection model (onnx), empty for person_detection_mediapipe_2023mar.onnx of opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``pose_estimation_model``
     - string
     - ""
     - yes
     - Path or URL of the MediaPipe pose estimation model (onnx), empty for pose_estimation_mediapipe_2023mar.onnx of opencv_zoo. The URL is downloaded to ~/.ros/opencv_apps/models/ at the first time
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers
   * - ``smoothing``
     - bool
     - true
     - yes
     - Smooth the landmarks of the tracked persons by One Euro filter
   * - ``smoothing_beta``
     - double
     - 80.0 (0 - 10000)
     - yes
     - Speed coefficient of One Euro filter, larger follows the fast motion with less lag
   * - ``smoothing_derivative_cutoff``
     - double
     - 1.0 (0.001 - 100)
     - yes
     - Cutoff frequency (Hz) of the velocity of One Euro filter
   * - ``smoothing_min_cutoff``
     - double
     - 0.05 (0.001 - 100)
     - yes
     - Minimum cutoff frequency (Hz) of One Euro filter, smaller is smoother when still
   * - ``tracking``
     - bool
     - true
     - yes
     - Track the persons by their poses in the previous image, as MediaPipe, and run the person detection every detection_interval images, otherwise every image

Source
------

`src/nodelet/dnn_pose_estimation_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/dnn_pose_estimation_nodelet.cpp>`_, 
`launch/dnn_pose_estimation.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/dnn_pose_estimation.launch.xml>`_
