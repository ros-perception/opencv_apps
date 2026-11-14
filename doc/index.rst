opencv_apps
===========

opencv_apps provides the ROS nodes that run the functions of `OpenCV <https://opencv.org/>`_: image processing,
features, contours, optical flows, tracking, detection and recognition, and the deep learning models of ``cv::dnn``
and `opencv_zoo <https://github.com/opencv/opencv_zoo>`_.

Each node subscribes an image topic, and publishes its results as messages (e.g.
``opencv_apps/msg/DetectionArrayStamped``) and as an image with the results drawn on it, so you can use the
functions of OpenCV just by running a launch file. The same source supports both ROS 1 and ROS 2.

What opencv_apps can do
-----------------------

The nodes are grouped by what they find, e.g. what the whole image shows (image classification), the bounding boxes of
the objects (object detection) or the regions of the pixels (segmentation). The nodes of a group publish the same kind
of results, mostly the same type of message on the same topic, so that one can replace another, e.g.
``face_detection`` by ``dnn_face_detection``. Each group lists the nodes by the distributions that they run on, and
each tile shows the main output of the node. Click a node to see its topics, parameters and usage.

.. include:: gallery.rst

.. toctree::
   :maxdepth: 2
   :hidden:

   nodes
   messages
   credits

Installation
------------

.. code-block:: bash

   # ROS 2 (jazzy, kilted, lyrical), ROS 1 (noetic), or ros-one-opencv-apps of ROS One
   sudo apt install ros-$ROS_DISTRO-opencv-apps

.. include:: distributions.rst

The page of each node shows the versions of opencv_apps, the distributions and OpenCV which the node needs, e.g. the
nodes added in opencv_apps 3.0.0 are not in noetic, and some deep learning nodes need a newer OpenCV, e.g. of
lyrical.

To build from the source, clone `ros-perception/opencv_apps <https://github.com/ros-perception/opencv_apps>`_ into
the workspace and build it with ``colcon build`` (ROS 2) or ``catkin build`` (ROS 1). The same source supports ROS 1
and ROS 2.

Usage
-----

Each node has a launch file of the same name, which remaps ``image`` to the topic of the camera. To try it with the
USB camera ``/dev/video0``, ``image_publisher`` (of image_pipeline, on all the distributions) publishes the images
of the camera, given its number instead of a file:

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-image-publisher
   # ROS 2, terminal 1: publish the camera as image (0: the number of the camera, /dev/video0)
   ros2 run image_publisher image_publisher_node 0 --ros-args -r image_raw:=image
   # ROS 2, terminal 2: run the node on it
   ros2 launch opencv_apps face_detection.launch.xml image:=image
   # ROS 1, terminal 1: publish the camera as camera/image_raw (0: /dev/video0)
   rosrun image_publisher image_publisher 0 __name:=camera
   # ROS 1, terminal 2: run the node on it
   roslaunch opencv_apps face_detection.launch image:=camera/image_raw use_opencv4:=true

The launch files show the results in a window of OpenCV (``debug_view:=false`` to disable it), and the nodes publish
them on e.g. ``/face_detection/image`` (``ros2 run rqt_image_view rqt_image_view /face_detection/image``). With the
other camera drivers, remap ``image`` to their topic, e.g. ``/image_raw`` of usb_cam (ROS 2) or
``/usb_cam/image_raw`` of usb_cam (ROS 1).

The nodes are also components (ROS 2) and nodelets (ROS 1), e.g. ``opencv_apps::FaceDetectionNodelet``, to run in
the same process as the camera driver.

.. _common-parameters:

Common parameters
-----------------

The parameters of all the nodes, except that face_recognition has neither ``use_camera_info`` nor ``debug_view`` (the
``debug_view`` argument of its launch file is for the face detection and the image viewer which it runs):

.. list-table::
   :header-rows: 1

   * - Name
     - Type
     - Default
     - Changed at run time
     - Description
   * - ``use_camera_info``
     - bool
     - false
     - yes, at the next subscription
     - Subscribe ``camera_info`` synchronized with ``image``, instead of ``image`` only. Whether the camera info is
       used depends on the node, see their pages: e.g. aruco_detection uses the camera model for the poses of the
       markers, while the most of the nodes process and publish the images in the same way.
   * - ``debug_view``
     - bool
     - false (true in the launch files)
     - no, restart the node
     - Show the result in a window of OpenCV (``cv::imshow``).
   * - ``always_subscribe``
     - bool
     - false
     - no, restart the node
     - Subscribe the input even when no node subscribes the outputs. The nodes subscribe the input only while their
       outputs are subscribed, otherwise.
   * - ``latch``
     - bool
     - false
     - no, restart the node
     - Publish the outputs with the transient local durability (latched).
   * - ``verbose_connection``
     - bool
     - false
     - no, restart the node
     - Print the connections of the outputs.

The parameters of the nodes are changed at run time with ``ros2 param set`` (ROS 2) or ``dynamic_reconfigure`` (ROS
1), except those read only at the start: the common ones above, and those marked "no, restart the node" on the pages
of the nodes (e.g. ``queue_size``, and ``face_cascade_name`` of face_detection).

Deep learning models
--------------------

The ``dnn_*`` nodes download their models from the URLs of the default models (e.g. of opencv_zoo) at the first
run, and keep them in ``~/.ros/opencv_apps/models/`` (``$ROS_HOME/opencv_apps/models/``). The model parameters (e.g.
``model``) take a path or a URL of other models. The ``backend`` parameter selects the backend and the target of
``cv::dnn``: ``auto`` (CUDA when OpenCV is built with CUDA and a CUDA device is found, otherwise the CPU), ``cpu``,
``opencl``, ``opencl_fp16``, ``cuda`` or ``cuda_fp16``.

The models are under their own licenses, e.g. Apache 2.0 for most of opencv_zoo and MIT for MiDaS, see the source of
each node for the URL of its models.

Demo videos
-----------

The demo videos are made of `Tears of Steel <https://mango.blender.org/>`_ (CC BY 3.0, Blender Foundation) and the
clips of Wikimedia Commons under the Creative Commons licenses, see :doc:`credits`.
