background_subtraction
======================

Separates the foreground from the background with the background subtractors of OpenCV: MOG2 and KNN, and MOG, GMG, CNT, GSOC and LSBP of opencv_contrib.

.. list-table::
   :stub-columns: 1

   * - opencv_apps
     - 3.0.0 and later
   * - ROS
     - ROS One, jazzy, kilted, lyrical and later
   * - OpenCV
     - the bgsegm module of opencv_contrib for MOG, GMG, CNT, GSOC and LSBP

.. image:: https://ros-perception.github.io/opencv_apps/media/background_subtraction.gif
   :alt: demo of background_subtraction

Usage
-----

.. code-block:: bash

   # ROS 2
   ros2 launch opencv_apps background_subtraction.launch.xml image:=/camera/image_raw debug_view:=false
   # ROS 1
   roslaunch opencv_apps background_subtraction.launch image:=/camera/image_raw debug_view:=false

Try it with the demo video
--------------------------

Download the input video of the demo above, publish it as ``image`` with ``image_publisher``, and run the node on it, in the terminals:

.. code-block:: bash

   wget https://ros-perception.github.io/opencv_apps/media/clips/pool.mp4
   # terminal 1: publish the video as image in a loop
   ros2 run image_publisher image_publisher_node pool.mp4 --ros-args -p publish_rate:=10.0 -r image_raw:=image
   # terminal 2: run the node with the parameters of the demo
   ros2 launch opencv_apps background_subtraction.launch.xml image:=image method:=0 node_name:=background_subtraction_mog2
   # terminal 3: view the result
   ros2 run rqt_image_view rqt_image_view /background_subtraction_mog2/image

MOG2; the camera is fixed and the balls move.

In ROS 1:

.. code-block:: bash

   # terminal 1
   rosrun image_publisher image_publisher pool.mp4 _publish_rate:=10 __name:=camera
   # terminal 2
   roslaunch opencv_apps background_subtraction.launch image:=camera/image_raw method:=0 node_name:=background_subtraction_mog2

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
   * - ``~/background``
     - ``sensor_msgs/msg/Image``
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
   * - ``cnt_max_pixel_stability``
     - int
     - 900 (1 - 10000)
     - yes
     - CNT: Maximum allowed credit for a pixel in history
   * - ``cnt_min_pixel_stability``
     - int
     - 15 (1 - 1000)
     - yes
     - CNT: Number of frames with same pixel color to consider stable
   * - ``cnt_use_history``
     - bool
     - true
     - yes
     - CNT: Give a pixel credit for being stable for a long time
   * - ``gmg_background_prior``
     - double
     - 0.8 (0 - 1)
     - yes
     - GMG: Prior probability that any given pixel is a background pixel
   * - ``gmg_decision_threshold``
     - double
     - 0.8 (0 - 1)
     - yes
     - GMG: Threshold of the probability above which a pixel is foreground
   * - ``gmg_initialization_frames``
     - int
     - 120 (1 - 1000)
     - yes
     - GMG: Number of frames used to initialize the background models
   * - ``gmg_max_features``
     - int
     - 64 (1 - 512)
     - yes
     - GMG: Total number of distinct colors to maintain in histogram (recreates the model)
   * - ``gmg_quantization_levels``
     - int
     - 16 (1 - 128)
     - yes
     - GMG: Number of discrete levels in each channel to be used in histograms (recreates the model)
   * - ``gmg_smoothing_radius``
     - int
     - 7 (0 - 31)
     - yes
     - GMG: Kernel size of the median filter to clean up the foreground mask, 0 (no filter) or odd
   * - ``gsoc_hits_threshold``
     - int
     - 32 (1 - 1000)
     - yes
     - GSOC: How many positives the sample must get before it will be considered as a possible replacement (recreates the model)
   * - ``gsoc_n_samples``
     - int
     - 20 (2 - 100)
     - yes
     - GSOC: Number of samples to maintain at each point of the frame (recreates the model)
   * - ``gsoc_propagation_rate``
     - double
     - 0.01 (0 - 1)
     - yes
     - GSOC: Probability of propagating to neighbors (recreates the model)
   * - ``gsoc_replace_rate``
     - double
     - 0.003 (0 - 1)
     - yes
     - GSOC: Probability of replacing the old sample (recreates the model)
   * - ``knn_detect_shadows``
     - bool
     - true
     - yes
     - KNN: Detect shadows and mark them (127) in the foreground mask
   * - ``knn_dist2_threshold``
     - double
     - 400.0 (1 - 5000)
     - yes
     - KNN: Threshold on the squared distance between the pixel and the sample to decide whether a pixel is close to that sample
   * - ``knn_history``
     - int
     - 500 (1 - 5000)
     - yes
     - KNN: Length of the history
   * - ``learning_rate``
     - double
     - -1.0 (-1 - 1)
     - yes
     - Learning rate of the background model, negative: automatically chosen, 0: not updated, 1: reinitialized from the last frame (CNT, GSOC and LSBP may still update the model at 0, and GMG, GSOC and LSBP do not reinitialize it at 1)
   * - ``lsbp_min_count``
     - int
     - 2 (1 - 100)
     - yes
     - LSBP: Minimal number of matches for a pixel to be background (recreates the model)
   * - ``lsbp_n_samples``
     - int
     - 20 (2 - 100)
     - yes
     - LSBP: Number of samples to maintain at each point of the frame (recreates the model)
   * - ``lsbp_radius``
     - int
     - 16 (1 - 64)
     - yes
     - LSBP: Radius of the LSBP descriptor (recreates the model)
   * - ``lsbp_threshold``
     - int
     - 8 (0 - 32)
     - yes
     - LSBP: Threshold of the LSBP descriptor distance (recreates the model)
   * - ``method``
     - int
     - 0 (0 - 6)
     - yes
     - Background subtraction method, 0: MOG2, 1: KNN, 2: MOG, 3: GMG, 4: CNT, 5: GSOC, 6: LSBP (MOG, GMG, CNT, GSOC and LSBP need the bgsegm module of opencv_contrib, otherwise MOG2 is used)
   * - ``mog2_detect_shadows``
     - bool
     - true
     - yes
     - MOG2: Detect shadows and mark them (127) in the foreground mask
   * - ``mog2_history``
     - int
     - 500 (1 - 5000)
     - yes
     - MOG2: Length of the history
   * - ``mog2_nmixtures``
     - int
     - 5 (1 - 10)
     - yes
     - MOG2: Number of Gaussian components in the background model (recreates the model)
   * - ``mog2_var_threshold``
     - double
     - 16.0 (1 - 256)
     - yes
     - MOG2: Threshold on the squared Mahalanobis distance between the pixel and the model to decide whether a pixel is well described by the background model
   * - ``mog_background_ratio``
     - double
     - 0.7 (0 - 1)
     - yes
     - MOG: Background ratio
   * - ``mog_history``
     - int
     - 200 (1 - 5000)
     - yes
     - MOG: Length of the history
   * - ``mog_nmixtures``
     - int
     - 5 (1 - 8)
     - yes
     - MOG: Number of Gaussian mixtures, at most 8 by OpenCV (recreates the model)
   * - ``mog_noise_sigma``
     - double
     - 0.0 (0 - 100)
     - yes
     - MOG: Noise strength, 0: automatic
   * - ``queue_size``
     - int
     - 3
     - no, restart the node
     - Specify queue_size of input image subscribers

Source
------

`src/nodelet/background_subtraction_nodelet.cpp <https://github.com/ros-perception/opencv_apps/blob/indigo/src/nodelet/background_subtraction_nodelet.cpp>`_, 
`launch/background_subtraction.launch.xml <https://github.com/ros-perception/opencv_apps/blob/indigo/launch/background_subtraction.launch.xml>`_
