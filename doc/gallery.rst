.. raw:: html

   <style>
   /* the tiles of the nodes on the top page (doc/tools/generate_pages.py) */
   .opencv-apps-tier {
     margin: 0 0 6px;
     font-size: 90%;
     font-weight: bold;
     color: #6a6a6a;
   }
   .opencv-apps-gallery {
     display: grid;
     grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
     gap: 12px;
     margin-bottom: 24px;
   }
   .opencv-apps-tile {
     display: block;
     border: 1px solid #e1e4e5;
     border-radius: 6px;
     overflow: hidden;
     text-align: center;
     color: #404040;
     background: #fff;
   }
   .opencv-apps-tile:hover {
     box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
   }
   .opencv-apps-tile img {
     display: block;
     width: 100%;
     aspect-ratio: 16 / 9;
     object-fit: cover;
     background: #222;
   }
   .opencv-apps-tile span {
     display: block;
     padding: 4px;
     font-family: monospace;
     font-size: 80%;
     overflow-wrap: anywhere;
   }
   /* wider pages for the tiles */
   .wy-nav-content {
     max-width: 1100px;
   }
   </style>
   <h3>Image filters</h3>
   <p>Filter and transform the images: edges, histogram equalization, morphology, image pyramids, smoothing, thresholds, the Fourier transform and the super resolution by a DNN.<br>Output: the processed image in <code>~/image</code>.</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/adding_images.html"><img src="https://ros-perception.github.io/opencv_apps/media/adding_images_thumb.gif" alt="adding_images" loading="lazy"><span>adding_images<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/discrete_fourier_transform.html"><img src="https://ros-perception.github.io/opencv_apps/media/discrete_fourier_transform_thumb.gif" alt="discrete_fourier_transform" loading="lazy"><span>discrete_fourier_transform<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/edge_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/edge_detection_thumb.gif" alt="edge_detection" loading="lazy"><span>edge_detection<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/pyramids.html"><img src="https://ros-perception.github.io/opencv_apps/media/pyramids_thumb.gif" alt="pyramids" loading="lazy"><span>pyramids<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/smoothing.html"><img src="https://ros-perception.github.io/opencv_apps/media/smoothing_thumb.gif" alt="smoothing" loading="lazy"><span>smoothing<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/threshold.html"><img src="https://ros-perception.github.io/opencv_apps/media/threshold_thumb.gif" alt="threshold" loading="lazy"><span>threshold<br><small>~/image</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/equalize_histogram.html"><img src="https://ros-perception.github.io/opencv_apps/media/equalize_histogram_thumb.gif" alt="equalize_histogram" loading="lazy"><span>equalize_histogram<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/morphology.html"><img src="https://ros-perception.github.io/opencv_apps/media/morphology_thumb.gif" alt="morphology" loading="lazy"><span>morphology<br><small>~/image</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 2: lyrical and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_super_resolution.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_super_resolution_thumb.gif" alt="dnn_super_resolution" loading="lazy"><span>dnn_super_resolution<br><small>~/image</small></span></a>
   </div>
   <h3>Features and shapes</h3>
   <p>Find the corners, the lines and the line segments, the circles and the blobs, and the contours with their bounding shapes, moments and convex hulls.<br>Output: <code>~/corners</code> (Point2DArrayStamped), <code>~/lines</code> (LineArrayStamped), <code>~/circles</code> (CircleArrayStamped) and <code>~/contours</code> (ContourArrayStamped, <code>~/hulls</code> of convex_hull), by the kind of the features.</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/corner_harris.html"><img src="https://ros-perception.github.io/opencv_apps/media/corner_harris_thumb.gif" alt="corner_harris" loading="lazy"><span>corner_harris<br><small>~/corners</small></span></a>
   <a class="opencv-apps-tile" href="nodes/goodfeature_track.html"><img src="https://ros-perception.github.io/opencv_apps/media/goodfeature_track_thumb.gif" alt="goodfeature_track" loading="lazy"><span>goodfeature_track<br><small>~/corners</small></span></a>
   <a class="opencv-apps-tile" href="nodes/hough_lines.html"><img src="https://ros-perception.github.io/opencv_apps/media/hough_lines_thumb.gif" alt="hough_lines" loading="lazy"><span>hough_lines<br><small>~/lines</small></span></a>
   <a class="opencv-apps-tile" href="nodes/hough_circles.html"><img src="https://ros-perception.github.io/opencv_apps/media/hough_circles_thumb.gif" alt="hough_circles" loading="lazy"><span>hough_circles<br><small>~/circles</small></span></a>
   <a class="opencv-apps-tile" href="nodes/find_contours.html"><img src="https://ros-perception.github.io/opencv_apps/media/find_contours_thumb.gif" alt="find_contours" loading="lazy"><span>find_contours<br><small>~/contours</small></span></a>
   <a class="opencv-apps-tile" href="nodes/general_contours.html"><img src="https://ros-perception.github.io/opencv_apps/media/general_contours_thumb.gif" alt="general_contours" loading="lazy"><span>general_contours<br><small>~/contours</small></span></a>
   <a class="opencv-apps-tile" href="nodes/contour_moments.html"><img src="https://ros-perception.github.io/opencv_apps/media/contour_moments_thumb.gif" alt="contour_moments" loading="lazy"><span>contour_moments<br><small>~/contours</small></span></a>
   <a class="opencv-apps-tile" href="nodes/convex_hull.html"><img src="https://ros-perception.github.io/opencv_apps/media/convex_hull_thumb.gif" alt="convex_hull" loading="lazy"><span>convex_hull<br><small>~/hulls</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/line_segment_detector.html"><img src="https://ros-perception.github.io/opencv_apps/media/line_segment_detector_thumb.gif" alt="line_segment_detector" loading="lazy"><span>line_segment_detector<br><small>~/lines</small></span></a>
   <a class="opencv-apps-tile" href="nodes/blob_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/blob_detection_thumb.gif" alt="blob_detection" loading="lazy"><span>blob_detection<br><small>~/circles</small></span></a>
   </div>
   <h3>Segmentation</h3>
   <p>Divide the image into regions by the class of each pixel: the pixels in a range of colors, the foreground, the connected regions, the classes of a DNN, and the object given by a box or points.<br>Output: the mask or label image, <code>~/image</code> of the color filters and background_subtraction, <code>~/label</code> of dnn_semantic_segmentation, <code>~/mask</code> of dnn_interactive_segmentation, and <code>~/contours</code> (ContourArrayStamped) of the regions of segment_objects, watershed_segmentation and dnn_interactive_segmentation.</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/hsv_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/hsv_color_filter_thumb.gif" alt="hsv_color_filter" loading="lazy"><span>hsv_color_filter<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/hls_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/hls_color_filter_thumb.gif" alt="hls_color_filter" loading="lazy"><span>hls_color_filter<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/rgb_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/rgb_color_filter_thumb.gif" alt="rgb_color_filter" loading="lazy"><span>rgb_color_filter<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/segment_objects.html"><img src="https://ros-perception.github.io/opencv_apps/media/segment_objects_thumb.gif" alt="segment_objects" loading="lazy"><span>segment_objects<br><small>~/contours</small></span></a>
   <a class="opencv-apps-tile" href="nodes/watershed_segmentation.html"><img src="https://ros-perception.github.io/opencv_apps/media/watershed_segmentation_thumb.gif" alt="watershed_segmentation" loading="lazy"><span>watershed_segmentation<br><small>~/contours</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/lab_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/lab_color_filter_thumb.gif" alt="lab_color_filter" loading="lazy"><span>lab_color_filter<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/background_subtraction.html"><img src="https://ros-perception.github.io/opencv_apps/media/background_subtraction_thumb.gif" alt="background_subtraction" loading="lazy"><span>background_subtraction<br><small>~/image</small></span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_semantic_segmentation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_semantic_segmentation_thumb.gif" alt="dnn_semantic_segmentation" loading="lazy"><span>dnn_semantic_segmentation<br><small>~/label</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 2: lyrical and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_interactive_segmentation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_interactive_segmentation_thumb.gif" alt="dnn_interactive_segmentation" loading="lazy"><span>dnn_interactive_segmentation<br><small>~/mask</small></span></a>
   </div>
   <h3>Image classification</h3>
   <p>Find what the whole image shows, the classes of ImageNet.<br>Output: <code>~/detections</code> (DetectionArrayStamped), the classes with their scores.</p>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_image_classification.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_image_classification_thumb.gif" alt="dnn_image_classification" loading="lazy"><span>dnn_image_classification<br><small>~/detections</small></span></a>
   </div>
   <h3>Object detection</h3>
   <p>Find the bounding boxes of the people (HOG of people_detect) and the objects (YOLO of dnn_object_detection: YOLOX, YOLOv4 and the other models of cv::dnn).<br>Output: <code>~/detections</code> (DetectionArrayStamped).</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/people_detect.html"><img src="https://ros-perception.github.io/opencv_apps/media/people_detect_thumb.gif" alt="people_detect" loading="lazy"><span>people_detect<br><small>~/detections</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_object_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_object_detection_thumb.gif" alt="dnn_object_detection" loading="lazy"><span>dnn_object_detection<br><small>~/detections</small></span></a>
   </div>
   <h3>Codes and markers</h3>
   <p>Find and decode QR codes, barcodes and ArUco markers.<br>Output: <code>~/detections</code> (DetectionArrayStamped), with the decoded data and the corners.</p>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/qrcode_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/qrcode_detection_thumb.gif" alt="qrcode_detection" loading="lazy"><span>qrcode_detection<br><small>~/detections</small></span></a>
   <a class="opencv-apps-tile" href="nodes/barcode_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/barcode_detection_thumb.gif" alt="barcode_detection" loading="lazy"><span>barcode_detection<br><small>~/detections</small></span></a>
   <a class="opencv-apps-tile" href="nodes/aruco_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/aruco_detection_thumb.gif" alt="aruco_detection" loading="lazy"><span>aruco_detection<br><small>~/detections</small></span></a>
   </div>
   <h3>Text detection and recognition</h3>
   <p>Find the texts and read them (OCR).<br>Output: <code>~/detections</code> (DetectionArrayStamped), with the recognized text.</p>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_text_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_text_detection_thumb.gif" alt="dnn_text_detection" loading="lazy"><span>dnn_text_detection<br><small>~/detections</small></span></a>
   </div>
   <h3>Faces</h3>
   <p>Detect and recognize the faces and their expressions.<br>Output: <code>~/faces</code> (FaceArrayStamped).</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/face_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/face_detection_thumb.gif" alt="face_detection" loading="lazy"><span>face_detection<br><small>~/faces</small></span></a>
   <a class="opencv-apps-tile" href="nodes/face_recognition.html"><img src="https://ros-perception.github.io/opencv_apps/media/face_recognition_thumb.gif" alt="face_recognition" loading="lazy"><span>face_recognition<br><small>~/faces</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_face_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_face_detection_thumb.gif" alt="dnn_face_detection" loading="lazy"><span>dnn_face_detection<br><small>~/faces</small></span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_face_recognition.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_face_recognition_thumb.gif" alt="dnn_face_recognition" loading="lazy"><span>dnn_face_recognition<br><small>~/faces</small></span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_facial_expression_recognition.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_facial_expression_recognition_thumb.gif" alt="dnn_facial_expression_recognition" loading="lazy"><span>dnn_facial_expression_recognition<br><small>~/faces</small></span></a>
   </div>
   <h3>Human and hand poses</h3>
   <p>Estimate the poses of the people and the hands.<br>Output: <code>~/skeletons</code> (SkeletonArrayStamped).</p>
   <p class="opencv-apps-tier">ROS 2: lyrical and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_pose_estimation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_pose_estimation_thumb.gif" alt="dnn_pose_estimation" loading="lazy"><span>dnn_pose_estimation<br><small>~/skeletons</small></span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_hand_pose_estimation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_hand_pose_estimation_thumb.gif" alt="dnn_hand_pose_estimation" loading="lazy"><span>dnn_hand_pose_estimation<br><small>~/skeletons</small></span></a>
   </div>
   <h3>Tracking</h3>
   <p>Track a target given by ``set_roi`` or the mouse.<br>Output: <code>~/output</code> (RectArrayStamped), <code>~/track_box</code> (RotatedRectStamped) of camshift.</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/camshift.html"><img src="https://ros-perception.github.io/opencv_apps/media/camshift_thumb.gif" alt="camshift" loading="lazy"><span>camshift<br><small>~/track_box</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/tracking.html"><img src="https://ros-perception.github.io/opencv_apps/media/tracking_thumb.gif" alt="tracking" loading="lazy"><span>tracking<br><small>~/output</small></span></a>
   <a class="opencv-apps-tile" href="nodes/match_template.html"><img src="https://ros-perception.github.io/opencv_apps/media/match_template_thumb.gif" alt="match_template" loading="lazy"><span>match_template<br><small>~/output</small></span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_tracking.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_tracking_thumb.gif" alt="dnn_tracking" loading="lazy"><span>dnn_tracking<br><small>~/output</small></span></a>
   </div>
   <h3>Optical flow and motion</h3>
   <p>Estimate the motion of the image.<br>Output: <code>~/flows</code> (FlowArrayStamped), <code>~/shift</code> (Point2DStamped) of phase_corr.</p>
   <p class="opencv-apps-tier">ROS 1: noetic and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/fback_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/fback_flow_thumb.gif" alt="fback_flow" loading="lazy"><span>fback_flow<br><small>~/flows</small></span></a>
   <a class="opencv-apps-tile" href="nodes/lk_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/lk_flow_thumb.gif" alt="lk_flow" loading="lazy"><span>lk_flow<br><small>~/flows</small></span></a>
   <a class="opencv-apps-tile" href="nodes/simple_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/simple_flow_thumb.gif" alt="simple_flow" loading="lazy"><span>simple_flow<br><small>~/flows</small></span></a>
   <a class="opencv-apps-tile" href="nodes/phase_corr.html"><img src="https://ros-perception.github.io/opencv_apps/media/phase_corr_thumb.gif" alt="phase_corr" loading="lazy"><span>phase_corr<br><small>~/shift</small></span></a>
   </div>
   <p class="opencv-apps-tier">ROS 2: lyrical and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_optical_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_optical_flow_thumb.gif" alt="dnn_optical_flow" loading="lazy"><span>dnn_optical_flow<br><small>~/flows</small></span></a>
   </div>
   <h3>Histogram comparison</h3>
   <p>Compare the color histogram of the image with a reference.<br>Output: <code>~/distance</code> (Float64).</p>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/compare_histogram.html"><img src="https://ros-perception.github.io/opencv_apps/media/compare_histogram_thumb.gif" alt="compare_histogram" loading="lazy"><span>compare_histogram<br><small>~/distance</small></span></a>
   </div>
   <h3>Depth and 3D localization</h3>
   <p>Estimate the depth of a color image by a DNN, and localize the objects, the faces, the tracked targets and the human and hand skeletons of the nodes above in 3D, with a depth image of an RGB-D camera or of the depth estimation, or a point cloud of a LiDAR, and publish their TF frames.<br>Output: <code>~/inverse_depth</code> or <code>~/depth</code> (Image) of dnn_depth_estimation, and <code>~/detections</code> (vision_msgs/Detection3DArray), <code>~/markers</code> (for rviz) and the TF frames of detection_3d.</p>
   <p><img src="https://raw.githubusercontent.com/ros-perception/opencv_apps/indigo/doc/images/detection_3d_pipeline.svg" alt="the pipeline of detection_3d" style="width: 100%; max-width: 900px"></p>
   <p><img src="https://raw.githubusercontent.com/ros-perception/opencv_apps/docs-media/detection_3d_skeletons_rviz.gif" alt="detection_3d of the human skeletons in rviz" style="width: 100%; max-width: 900px"></p>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_depth_estimation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_depth_estimation_thumb.gif" alt="dnn_depth_estimation" loading="lazy"><span>dnn_depth_estimation<br><small>~/inverse_depth</small></span></a>
   <a class="opencv-apps-tile" href="nodes/detection_3d.html"><img src="https://ros-perception.github.io/opencv_apps/media/detection_3d_thumb.gif" alt="detection_3d" loading="lazy"><span>detection_3d<br><small>~/detections</small></span></a>
   </div>
   <h3>Odometry and maps</h3>
   <p>Estimate the motion of the robot by an RGB-D camera or a LiDAR (e.g. Livox), as the odometry for slam_toolbox (the 2D map) and Nav2 instead of the odometry of the wheels, and build the 3D map and the 3D trajectory.<br>Output: <code>~/odom</code> (nav_msgs/Odometry), the TF odom -> base_frame, <code>~/path</code> (nav_msgs/Path) and <code>~/map</code> (PointCloud2, the 3D map).</p>
   <p class="opencv-apps-tier">ROS 1: ROS One and later, ROS 2: jazzy and later</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/rgbd_odometry.html"><img src="https://ros-perception.github.io/opencv_apps/media/rgbd_odometry_thumb.gif" alt="rgbd_odometry" loading="lazy"><span>rgbd_odometry<br><small>~/odom, ~/map</small></span></a>
   <a class="opencv-apps-tile" href="nodes/laser_odometry.html"><img src="https://ros-perception.github.io/opencv_apps/media/laser_odometry_thumb.gif" alt="laser_odometry" loading="lazy"><span>laser_odometry<br><small>~/odom, ~/map</small></span></a>
   </div>
