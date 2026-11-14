face_recognition
================

Recognizes the faces detected by :doc:`face_detection` with the face recognizers of opencv_contrib (Eigenfaces, Fisherfaces or LBPH), trained with the ``~/train`` service.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 1.12.0 and later (ROS 1), 3.0.0 and later (ROS 2)
   * - ROS
     - noetic, ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - any version of the distributions
   * - Group
     - Faces, the nodes of which publish ``~/faces`` (FaceArrayStamped)

.. image:: https://ros-perception.github.io/opencv_apps/media/face_recognition.gif
   :alt: demo of face_recognition

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps face_recognition.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps face_recognition.launch image:=/camera/image_raw debug_view:=false use_opencv4:=true

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/face_detector_withface_test_diamondback.mp4
   wget https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/test/face_data.tar.gz
   tar xzf face_data.tar.gz
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node face_detector_withface_test_diamondback.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps face_recognition.launch.xml image:=image data_dir:=$PWD/face_data launch_face_detection:=true launch_trainer:=false
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /face_recognition/debug_image

The faces trained in test/face_data.tar.gz; ~debug_image is shown.

See :doc:`../credits` for the license of the video.

Subscribed topics
-----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
   * - ``faces``
     - :ref:`opencv_apps/msg/FaceArrayStamped <msg-FaceArrayStamped>`
   * - ``image``
     - ``sensor_msgs/msg/Image``

Published topics
----------------

.. list-table::
   :header-rows: 1

   * - Topic
     - Type
     - Note
   * - ``~/debug_image``
     - ``sensor_msgs/msg/Image``
     -  
   * - ``~/faces`` **(recommended)**
     - :ref:`opencv_apps/msg/FaceArrayStamped <msg-FaceArrayStamped>`
     - Recommended, opencv_apps 3.0.1 and later. The same as ``~/output``, with the name of the other face nodes.
   * - ``~/output`` (compatibility)
     - :ref:`opencv_apps/msg/FaceArrayStamped <msg-FaceArrayStamped>`
     - Kept for compatibility, use ``~/faces``.

Services
--------

.. list-table::
   :header-rows: 1

   * - Service
     - Type
   * - ``~/train``
     - ``opencv_apps/srv/FaceRecognitionTrain``

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
     -  
   * - ``data_dir``
     - string
     - "~/.ros/opencv_apps/face_data"
     - yes
     - Save directory for train data
   * - ``face_model_height``
     - int
     - 90 (30 - 500)
     - yes
     - Height of training face image
   * - ``face_model_width``
     - int
     - 190 (30 - 500)
     - yes
     - Width of training face image
   * - ``face_padding``
     - double
     - 0.1 (0 - 2)
     - yes
     - Padding ratio of each face
   * - ``lbph_grid_x``
     - int
     - 8 (1 - 30)
     - yes
     - grid_x parameter used only for LBPH model
   * - ``lbph_grid_y``
     - int
     - 8 (1 - 30)
     - yes
     - grid_y parameter used only for LBPH model
   * - ``lbph_neighbors``
     - int
     - 8 (1 - 30)
     - yes
     - Neighbors parameter used only for LBPH model
   * - ``lbph_radius``
     - int
     - 1 (1 - 10)
     - yes
     - Radius parameter used only for LBPH model
   * - ``model_method``
     - string
     - "eigen"
     - yes
     - Method to recognize faces, "eigen": eigen, "fisher": fisher, "LBPH": LBPH
   * - ``model_num_components``
     - int
     - 0 (0 - 100)
     - yes
     - Number of components for face recognizer model
   * - ``model_threshold``
     - double
     - 8000.0 (0 - 10000)
     - yes
     - Threshold for face recognizer model
   * - ``queue_size``
     - int
     - 100
     - no, restart the node
     - Specigy queue_size of input image subscribers
   * - ``save_train_data``
     - bool
     - true
     - yes
     - Save train data
   * - ``use_saved_data``
     - bool
     - true
     - yes
     - Use saved data

Source
------

`src/nodelet/face_recognition_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/face_recognition_nodelet.cpp>`_, 
`launch/face_recognition.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/face_recognition.launch.xml>`_
