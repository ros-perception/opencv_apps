detection_3d
============

Publishes the 3D boxes (``vision_msgs/Detection3DArray``) and the TF frames ``<label>_<id>`` of the 2D results of the other nodes, ``~/detections`` of the detection nodes (``input_type:=detections``), ``~/faces`` of the face nodes (``faces``) or ``~/output`` of the tracking nodes (``rects``), with a depth image registered to their color image, e.g. of an RGB-D camera (``depth_source:=depth``), or a point cloud, e.g. of a LiDAR, transformed to the camera frame by TF (``depth_source:=points``). The points of a 2D box are the pixels of the depth image in the box or the points of the cloud projected into the box, the depth of the object is the median of their depths, and its 3D box is the extent of the points within ``depth_threshold`` of the median, as `yolo_ros <https://github.com/mgonzs13/yolo_ros>`_. The objects are tracked by the distance of their centers, so that each object keeps its TF frame. On ROS 1, the results of vision_msgs (0.0.2) have only the class id of the detections, and the labels are in the TF frames.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.1.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - 3D localization, the nodes of which publish ``~/detections`` (vision_msgs/Detection3DArray) and the TF frames

.. image:: https://ros-perception.github.io/opencv_apps/media/detection_3d.gif
   :alt: demo of detection_3d

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps detection_3d.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps detection_3d.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input of the demo above, publish it, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/publish_rgbd.py
   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/rgbd_walking.mp4
   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/rgbd_walking_depth.npz
   # terminal 1: publish the input in a loop
   python3 publish_rgbd.py rgbd_walking.mp4 rgbd_walking_depth.npz
   # terminal 2: run the node
   ros2 launch opencv_apps detection_3d.launch.xml input:=/dnn_object_detection/detections depth:=depth camera_info:=camera_info
   # terminal 3: run, keep it running
   ros2 launch opencv_apps dnn_object_detection.launch.xml image:=image
   # terminal 4: view the result
   ros2 run rviz2 rviz2 -f camera_rgb_optical_frame

The input is 20 RGB-D images of the TUM RGB-D freiburg3_walking_xyz sequence (image, depth in mm and camera_info, by test/publish_rgbd.py), and the people of dnn_object_detection. In rviz2, add TF to show the frames person_0, ..., or add Detection3DArray of vision_msgs_rviz_plugins to show the boxes.

See :doc:`../credits` for the license of the video.

Sources of the distances
------------------------

The demos draw the 3D boxes of ``~/detections`` on the image, and their top view.

RGB-D camera (default)
^^^^^^^^^^^^^^^^^^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/detection_3d.gif
   :alt: demo of detection_3d with rgbd

.. code-block:: bash

   ros2 launch opencv_apps detection_3d.launch.xml input:=/dnn_object_detection/detections depth:=depth camera_info:=camera_info

The depth image of the TUM RGB-D dataset, as above.

LiDAR
^^^^^

.. image:: https://ros-perception.github.io/opencv_apps/media/detection_3d_lidar.gif
   :alt: demo of detection_3d with lidar

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/publish_lidar.py
   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/lidar_boreas.mp4
   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/lidar_boreas_points.npz
   python3 publish_lidar.py lidar_boreas.mp4 lidar_boreas_points.npz &
   ros2 launch opencv_apps dnn_object_detection.launch.xml image:=image &
   ros2 launch opencv_apps detection_3d.launch.xml input:=/dnn_object_detection/detections points:=points camera_info:=camera_info depth_source:=points depth_threshold:=1.0 max_depth:=60.0 track_distance:=2.0

20 images and the points of the LiDAR (in the frame lidar, with the static TF of the extrinsics from the frame camera) of the Boreas dataset, by test/publish_lidar.py; the objects are farther and move faster than indoors, so max_depth, depth_threshold and track_distance are larger.

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``camera_info``
     - ``sensor_msgs/msg/CameraInfo``
   * - ``depth``
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
     - ``vision_msgs/msg/Detection3DArray``
   * - ``/tf``
     - ``tf2_msgs/msg/TFMessage``

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
   * - ``depth_source``
     - string
     - "depth"
     - no, restart the node
     - Source of the distances: depth (the depth image registered to the color image, 16UC1 in mm or 32FC1 in m, e.g. of an RGB-D camera) or points (a point cloud, e.g. of a LiDAR, transformed to the frame of camera_info by TF).
   * - ``depth_threshold``
     - double
     - 0.3 (0.01 - 10)
     - yes
     - The points within this distance (m) of the median depth of the box belong to the object.
   * - ``input_type``
     - string
     - "detections"
     - no, restart the node
     - Type of input: detections (opencv_apps/DetectionArrayStamped, e.g. ~/detections of dnn_object_detection), faces (FaceArrayStamped, ~/faces of the face nodes) or rects (RectArrayStamped, ~/output of the tracking nodes).
   * - ``max_depth``
     - double
     - 10.0 (0 - 1000)
     - yes
     - Maximum valid depth (m).
   * - ``min_depth``
     - double
     - 0.1 (0 - 100)
     - yes
     - Minimum valid depth (m).
   * - ``publish_tf``
     - bool
     - true
     - yes
     - Publish the TF frame <tf_prefix><label>_<id> of each object.
   * - ``queue_size``
     - int
     - 10
     - no, restart the node
     - Queue size of the synchronized subscribers.
   * - ``roi_scale``
     - double
     - 0.8 (0.1 - 1)
     - yes
     - Scale of the 2D box whose depths are used, smaller to leave out the background at the edges of the box.
   * - ``tf_prefix``
     - string
     - ""
     - yes
     - Prefix of the TF frames.
   * - ``track_distance``
     - double
     - 0.5 (0 - 10)
     - yes
     - An object within this distance (m) of an object of the same label in the previous images is the same object, and keeps its id.
   * - ``track_lost_frames``
     - int
     - 10 (0 - 1000)
     - yes
     - Number of images that a lost object keeps its id for.

Source
------

`src/nodelet/detection_3d_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/detection_3d_nodelet.cpp>`_, 
`launch/detection_3d.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/detection_3d.launch.xml>`_
