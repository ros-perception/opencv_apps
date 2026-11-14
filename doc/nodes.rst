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

Features and shapes
-------------------

Find the corners, the lines and the line segments, the circles and the blobs, and the contours with their bounding shapes, moments and convex hulls. Output: ``~/corners`` (Point2DArrayStamped), ``~/lines`` (LineArrayStamped), ``~/circles`` (CircleArrayStamped) and ``~/contours`` (ContourArrayStamped, ``~/hulls`` of convex_hull), by the kind of the features.

.. toctree::
   :maxdepth: 1

   nodes/corner_harris
   nodes/goodfeature_track
   nodes/hough_lines
   nodes/line_segment_detector
   nodes/hough_circles
   nodes/blob_detection
   nodes/find_contours
   nodes/general_contours
   nodes/contour_moments
   nodes/convex_hull

Segmentation
------------

Divide the image into regions by the class of each pixel: the pixels in a range of colors, the foreground, the connected regions, and the classes of a DNN. Output: the mask or label image, ``~/image`` of the color filters and background_subtraction, ``~/label`` of dnn_semantic_segmentation, and ``~/contours`` (ContourArrayStamped) of the regions of segment_objects and watershed_segmentation.

.. toctree::
   :maxdepth: 1

   nodes/hsv_color_filter
   nodes/hls_color_filter
   nodes/lab_color_filter
   nodes/rgb_color_filter
   nodes/background_subtraction
   nodes/segment_objects
   nodes/watershed_segmentation
   nodes/dnn_semantic_segmentation

Image classification
--------------------

Find what the whole image shows, the classes of ImageNet. Output: ``~/detections`` (DetectionArrayStamped), the classes with their scores.

.. toctree::
   :maxdepth: 1

   nodes/dnn_image_classification

Object detection
----------------

Find the bounding boxes of the people and the objects. Output: ``~/detections`` (DetectionArrayStamped).

.. toctree::
   :maxdepth: 1

   nodes/people_detect
   nodes/dnn_object_detection

Codes and markers
-----------------

Find and decode QR codes, barcodes and ArUco markers. Output: ``~/detections`` (DetectionArrayStamped), with the decoded data and the corners.

.. toctree::
   :maxdepth: 1

   nodes/qrcode_detection
   nodes/barcode_detection
   nodes/aruco_detection

Text detection and recognition
------------------------------

Find the texts and read them (OCR). Output: ``~/detections`` (DetectionArrayStamped), with the recognized text.

.. toctree::
   :maxdepth: 1

   nodes/dnn_text_detection

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
