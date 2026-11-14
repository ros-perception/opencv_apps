Nodes
=====

Each node subscribes ``image`` (``sensor_msgs/msg/Image``), and publishes its results in the namespace of the node, e.g. ``edge_detection/image``. Remap ``image`` to the topic of the camera.

Image filters
-------------

Filter and transform the images: edges, histogram equalization, morphology, image pyramids, smoothing, thresholds, the Fourier transform and the depth estimated by a DNN. Output: the processed image in ``~/image``.

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
   nodes/dnn_depth_estimation

Color and region masks
----------------------

Extract the regions of the image: the pixels in a range of colors, the foreground, and the classes of a DNN segmentation. Output: the mask or label image, ``~/image`` of the color filters and background_subtraction, ``~/label`` of dnn_semantic_segmentation.

.. toctree::
   :maxdepth: 1

   nodes/hsv_color_filter
   nodes/hls_color_filter
   nodes/lab_color_filter
   nodes/rgb_color_filter
   nodes/background_subtraction
   nodes/dnn_semantic_segmentation

Corners
-------

Find the corners. Output: ``~/corners`` (Point2DArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/corner_harris
   nodes/goodfeature_track

Lines
-----

Find the lines and the line segments. Output: ``~/lines`` (LineArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/hough_lines
   nodes/line_segment_detector

Circles and blobs
-----------------

Find the circles and the blobs. Output: ``~/circles`` (CircleArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/hough_circles
   nodes/blob_detection

Contours
--------

Find the contours, with their bounding shapes, moments and convex hulls, and segment the objects. Output: ``~/contours`` (ContourArrayStamped), ``~/hulls`` of convex_hull.

.. toctree::
   :maxdepth: 1

   nodes/find_contours
   nodes/general_contours
   nodes/contour_moments
   nodes/convex_hull
   nodes/segment_objects
   nodes/watershed_segmentation

Detection
---------

Detect the people and the objects, decode QR codes, barcodes and ArUco markers, find texts, and classify the image. Output: ``~/detections`` (DetectionArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/people_detect
   nodes/qrcode_detection
   nodes/barcode_detection
   nodes/aruco_detection
   nodes/dnn_object_detection
   nodes/dnn_text_detection
   nodes/dnn_image_classification

Faces
-----

Detect and recognize the faces and their expressions. Output: ``~/faces`` (FaceArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/face_detection
   nodes/face_recognition
   nodes/dnn_face_detection
   nodes/dnn_face_recognition
   nodes/dnn_facial_expression_recognition

Human and hand poses
--------------------

Estimate the poses of the people and the hands. Output: ``~/skeletons`` (SkeletonArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/dnn_pose_estimation
   nodes/dnn_hand_pose_estimation

Tracking
--------

Track a target given by ``set_roi`` or the mouse. Output: ``~/output`` (RectArrayStamped), ``~/track_box`` (RotatedRectStamped) of camshift.

.. toctree::
   :maxdepth: 1

   nodes/tracking
   nodes/match_template
   nodes/dnn_tracking
   nodes/camshift

Optical flow and motion
-----------------------

Estimate the motion of the image. Output: ``~/flows`` (FlowArrayStamped), ``~/shift`` (Point2DStamped) of phase_corr.

.. toctree::
   :maxdepth: 1

   nodes/fback_flow
   nodes/lk_flow
   nodes/simple_flow
   nodes/phase_corr

Histogram comparison
--------------------

Compare the color histogram of the image with a reference. Output: ``~/distance`` (Float64).

.. toctree::
   :maxdepth: 1

   nodes/compare_histogram
