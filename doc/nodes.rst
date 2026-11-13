Nodes
=====

Each node subscribes ``image`` (``sensor_msgs/msg/Image``), and publishes its results in the namespace of the node, e.g. ``edge_detection/image``. Remap ``image`` to the topic of the camera.

Image processing
----------------

.. toctree::
   :maxdepth: 1

   nodes/adding_images
   nodes/discrete_fourier_transform
   nodes/edge_detection
   nodes/equalize_histogram
   nodes/morphology
   nodes/pyramids
   nodes/smoothing
   nodes/threshold

Color
-----

.. toctree::
   :maxdepth: 1

   nodes/hsv_color_filter
   nodes/hls_color_filter
   nodes/lab_color_filter
   nodes/rgb_color_filter
   nodes/compare_histogram

Features, lines and contours
----------------------------

.. toctree::
   :maxdepth: 1

   nodes/corner_harris
   nodes/goodfeature_track
   nodes/hough_lines
   nodes/hough_circles
   nodes/line_segment_detector
   nodes/find_contours
   nodes/general_contours
   nodes/contour_moments
   nodes/convex_hull
   nodes/blob_detection

Segmentation
------------

.. toctree::
   :maxdepth: 1

   nodes/segment_objects
   nodes/watershed_segmentation
   nodes/background_subtraction

Motion and tracking
-------------------

.. toctree::
   :maxdepth: 1

   nodes/fback_flow
   nodes/lk_flow
   nodes/simple_flow
   nodes/phase_corr
   nodes/camshift
   nodes/tracking
   nodes/match_template

Detection and recognition
-------------------------

.. toctree::
   :maxdepth: 1

   nodes/face_detection
   nodes/face_recognition
   nodes/people_detect
   nodes/qrcode_detection
   nodes/barcode_detection
   nodes/aruco_detection

Deep learning (cv::dnn)
-----------------------

.. toctree::
   :maxdepth: 1

   nodes/dnn_face_detection
   nodes/dnn_face_recognition
   nodes/dnn_facial_expression_recognition
   nodes/dnn_object_detection
   nodes/dnn_image_classification
   nodes/dnn_semantic_segmentation
   nodes/dnn_depth_estimation
   nodes/dnn_text_detection
   nodes/dnn_pose_estimation
   nodes/dnn_hand_pose_estimation
   nodes/dnn_tracking
