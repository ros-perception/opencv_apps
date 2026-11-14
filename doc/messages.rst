Messages and services
=====================

The messages and the services of opencv_apps, published and provided by the nodes.

.. _msg-Circle:

opencv_apps/msg/Circle
----------------------

.. code-block:: text

   Point2D center
   float64 radius


.. _msg-CircleArray:

opencv_apps/msg/CircleArray
---------------------------

.. code-block:: text

   Circle[] circles


.. _msg-CircleArrayStamped:

opencv_apps/msg/CircleArrayStamped
----------------------------------

.. code-block:: text

   std_msgs/Header header
   Circle[] circles


.. _msg-Contour:

opencv_apps/msg/Contour
-----------------------

.. code-block:: text

   Point2D[] points

.. _msg-ContourArray:

opencv_apps/msg/ContourArray
----------------------------

.. code-block:: text

   Contour[] contours

.. _msg-ContourArrayStamped:

opencv_apps/msg/ContourArrayStamped
-----------------------------------

.. code-block:: text

   std_msgs/Header header
   Contour[] contours


.. _msg-Detection:

opencv_apps/msg/Detection
-------------------------

.. code-block:: text

   # an object detected by dnn_object_detection or people_detect, or a code or a marker detected by qrcode_detection,
   # barcode_detection and aruco_detection
   string label        # class name of the object, QR_CODE, type of the barcode (e.g. EAN_13), or dictionary of the marker (e.g. DICT_4X4_50)
   string data         # decoded text of the code, or empty
   int32 id            # class id of the object, or id of the marker, or -1
   float64 score       # confidence of the detection, 1.0 if not given
   Rect rect           # bounding box, x-y is center point
   Point2D[] corners   # corners of the code or the marker, empty for the objects

.. _msg-DetectionArray:

opencv_apps/msg/DetectionArray
------------------------------

.. code-block:: text

   Detection[] detections

.. _msg-DetectionArrayStamped:

opencv_apps/msg/DetectionArrayStamped
-------------------------------------

.. code-block:: text

   std_msgs/Header header
   Detection[] detections

.. _msg-Face:

opencv_apps/msg/Face
--------------------

.. code-block:: text

   Rect face
   Rect[] eyes
   string label
   float64 confidence

.. _msg-FaceArray:

opencv_apps/msg/FaceArray
-------------------------

.. code-block:: text

   Face[] faces


.. _msg-FaceArrayStamped:

opencv_apps/msg/FaceArrayStamped
--------------------------------

.. code-block:: text

   std_msgs/Header header
   Face[] faces


.. _msg-Flow:

opencv_apps/msg/Flow
--------------------

.. code-block:: text

   Point2D point
   Point2D velocity

.. _msg-FlowArray:

opencv_apps/msg/FlowArray
-------------------------

.. code-block:: text

   Flow[] flow
   bool[] status

.. _msg-FlowArrayStamped:

opencv_apps/msg/FlowArrayStamped
--------------------------------

.. code-block:: text

   std_msgs/Header header
   Flow[] flow
   bool[] status

.. _msg-FlowStamped:

opencv_apps/msg/FlowStamped
---------------------------

.. code-block:: text

   std_msgs/Header header
   Flow flow

.. _msg-Keypoint:

opencv_apps/msg/Keypoint
------------------------

.. code-block:: text

   # a keypoint of a skeleton, e.g. a joint of a person estimated by dnn_pose_estimation
   string name        # name of the keypoint, e.g. nose, left_shoulder
   float64 x          # position in the image
   float64 y
   float64 visibility # probability that the keypoint is visible (not occluded) in the image, 0 to 1
   float64 presence   # probability that the keypoint is within the image, 0 to 1

.. _msg-Line:

opencv_apps/msg/Line
--------------------

.. code-block:: text

   Point2D pt1
   Point2D pt2


.. _msg-LineArray:

opencv_apps/msg/LineArray
-------------------------

.. code-block:: text

   Line[] lines

.. _msg-LineArrayStamped:

opencv_apps/msg/LineArrayStamped
--------------------------------

.. code-block:: text

   std_msgs/Header header
   Line[] lines

.. _msg-Moment:

opencv_apps/msg/Moment
----------------------

.. code-block:: text

   # spatial moments
   float64 m00
   float64 m10
   float64 m01
   float64 m20
   float64 m11
   float64 m02
   float64 m30
   float64 m21
   float64 m12
   float64 m03

   # central moments
   float64 mu20
   float64 mu11
   float64 mu02
   float64 mu30
   float64 mu21
   float64 mu12
   float64 mu03

   # central normalized moments
   float64 nu20
   float64 nu11
   float64 nu02
   float64 nu30
   float64 nu21
   float64 nu12
   float64 nu03

   # center of mass m10/m00, m01/m00
   Point2D center
   float64 length
   float64 area

.. _msg-MomentArray:

opencv_apps/msg/MomentArray
---------------------------

.. code-block:: text

   Moment[] moments

.. _msg-MomentArrayStamped:

opencv_apps/msg/MomentArrayStamped
----------------------------------

.. code-block:: text

   std_msgs/Header header
   Moment[] moments

.. _msg-Point2D:

opencv_apps/msg/Point2D
-----------------------

.. code-block:: text

   float64 x
   float64 y


.. _msg-Point2DArray:

opencv_apps/msg/Point2DArray
----------------------------

.. code-block:: text

   Point2D[] points

.. _msg-Point2DArrayStamped:

opencv_apps/msg/Point2DArrayStamped
-----------------------------------

.. code-block:: text

   std_msgs/Header header
   Point2D[] points

.. _msg-Point2DStamped:

opencv_apps/msg/Point2DStamped
------------------------------

.. code-block:: text

   std_msgs/Header header
   Point2D point


.. _msg-Rect:

opencv_apps/msg/Rect
--------------------

.. code-block:: text

   # opencv Rect data type, x-y is center point
   float64 x
   float64 y
   float64 width
   float64 height


.. _msg-RectArray:

opencv_apps/msg/RectArray
-------------------------

.. code-block:: text

   Rect[] rects

.. _msg-RectArrayStamped:

opencv_apps/msg/RectArrayStamped
--------------------------------

.. code-block:: text

   std_msgs/Header header
   Rect[] rects



.. _msg-RotatedRect:

opencv_apps/msg/RotatedRect
---------------------------

.. code-block:: text

   float64 angle
   Point2D center
   Size size

.. _msg-RotatedRectArray:

opencv_apps/msg/RotatedRectArray
--------------------------------

.. code-block:: text

   RotatedRect[] rects

.. _msg-RotatedRectArrayStamped:

opencv_apps/msg/RotatedRectArrayStamped
---------------------------------------

.. code-block:: text

   std_msgs/Header header
   RotatedRect[] rects


.. _msg-RotatedRectStamped:

opencv_apps/msg/RotatedRectStamped
----------------------------------

.. code-block:: text

   std_msgs/Header header
   RotatedRect rect



.. _msg-Size:

opencv_apps/msg/Size
--------------------

.. code-block:: text

   float64 width
   float64 height


.. _msg-Skeleton:

opencv_apps/msg/Skeleton
------------------------

.. code-block:: text

   # a skeleton (the keypoints of a pose), e.g. a person estimated by dnn_pose_estimation
   string label          # class of the skeleton, e.g. person
   int32 id              # id of the skeleton, the same while it is tracked over the images, or -1
   float64 score         # confidence of the pose
   Rect rect             # bounding box of the keypoints, x-y is center point
   Keypoint[] keypoints  # keypoints in the order of the model, e.g. the 33 landmarks of MediaPipe

.. _msg-SkeletonArray:

opencv_apps/msg/SkeletonArray
-----------------------------

.. code-block:: text

   Skeleton[] skeletons

.. _msg-SkeletonArrayStamped:

opencv_apps/msg/SkeletonArrayStamped
------------------------------------

.. code-block:: text

   std_msgs/Header header
   Skeleton[] skeletons

.. _msg-FaceRecognitionTrain:

opencv_apps/srv/FaceRecognitionTrain
------------------------------------

.. code-block:: text

   sensor_msgs/Image[] images
   Rect[] rects
   string[] labels
   ---
   bool ok
   string error

.. _msg-SetROI:

opencv_apps/srv/SetROI
----------------------

.. code-block:: text

   # Set the region of interest, e.g. the target of the tracking node.
   # The image on which the rect is given. If it is empty (no data), the rect is set on the next input image.
   sensor_msgs/Image image
   # The region of interest, x-y is its center point as in opencv_apps/Rect.
   Rect rect
   ---
   bool ok
   string error
