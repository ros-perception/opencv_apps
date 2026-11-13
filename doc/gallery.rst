.. raw:: html

   <style>
   /* the tiles of the nodes on the top page (doc/tools/generate_pages.py) */
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
   <h3>Image processing</h3>
   <p>Filter and transform the images: edges, histogram equalization, morphology, image pyramids, smoothing, thresholds and the Fourier transform.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/adding_images.html"><img src="https://ros-perception.github.io/opencv_apps/media/adding_images_thumb.gif" alt="adding_images" loading="lazy"><span>adding_images</span></a>
   <a class="opencv-apps-tile" href="nodes/discrete_fourier_transform.html"><img src="https://ros-perception.github.io/opencv_apps/media/discrete_fourier_transform_thumb.gif" alt="discrete_fourier_transform" loading="lazy"><span>discrete_fourier_transform</span></a>
   <a class="opencv-apps-tile" href="nodes/edge_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/edge_detection_thumb.gif" alt="edge_detection" loading="lazy"><span>edge_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/equalize_histogram.html"><img src="https://ros-perception.github.io/opencv_apps/media/equalize_histogram_thumb.gif" alt="equalize_histogram" loading="lazy"><span>equalize_histogram</span></a>
   <a class="opencv-apps-tile" href="nodes/morphology.html"><img src="https://ros-perception.github.io/opencv_apps/media/morphology_thumb.gif" alt="morphology" loading="lazy"><span>morphology</span></a>
   <a class="opencv-apps-tile" href="nodes/pyramids.html"><img src="https://ros-perception.github.io/opencv_apps/media/pyramids_thumb.gif" alt="pyramids" loading="lazy"><span>pyramids</span></a>
   <a class="opencv-apps-tile" href="nodes/smoothing.html"><img src="https://ros-perception.github.io/opencv_apps/media/smoothing_thumb.gif" alt="smoothing" loading="lazy"><span>smoothing</span></a>
   <a class="opencv-apps-tile" href="nodes/threshold.html"><img src="https://ros-perception.github.io/opencv_apps/media/threshold_thumb.gif" alt="threshold" loading="lazy"><span>threshold</span></a>
   </div>
   <h3>Color</h3>
   <p>Extract the pixels in a range of colors in HSV, HLS, L*a*b* or RGB, and compare the color histograms.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/hsv_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/hsv_color_filter_thumb.gif" alt="hsv_color_filter" loading="lazy"><span>hsv_color_filter</span></a>
   <a class="opencv-apps-tile" href="nodes/hls_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/hls_color_filter_thumb.gif" alt="hls_color_filter" loading="lazy"><span>hls_color_filter</span></a>
   <a class="opencv-apps-tile" href="nodes/lab_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/lab_color_filter_thumb.gif" alt="lab_color_filter" loading="lazy"><span>lab_color_filter</span></a>
   <a class="opencv-apps-tile" href="nodes/rgb_color_filter.html"><img src="https://ros-perception.github.io/opencv_apps/media/rgb_color_filter_thumb.gif" alt="rgb_color_filter" loading="lazy"><span>rgb_color_filter</span></a>
   <a class="opencv-apps-tile" href="nodes/compare_histogram.html"><img src="https://ros-perception.github.io/opencv_apps/media/compare_histogram_thumb.gif" alt="compare_histogram" loading="lazy"><span>compare_histogram</span></a>
   </div>
   <h3>Features, lines and contours</h3>
   <p>Find the corners, lines, line segments, circles, blobs and contours, with their moments and convex hulls.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/corner_harris.html"><img src="https://ros-perception.github.io/opencv_apps/media/corner_harris_thumb.gif" alt="corner_harris" loading="lazy"><span>corner_harris</span></a>
   <a class="opencv-apps-tile" href="nodes/goodfeature_track.html"><img src="https://ros-perception.github.io/opencv_apps/media/goodfeature_track_thumb.gif" alt="goodfeature_track" loading="lazy"><span>goodfeature_track</span></a>
   <a class="opencv-apps-tile" href="nodes/hough_lines.html"><img src="https://ros-perception.github.io/opencv_apps/media/hough_lines_thumb.gif" alt="hough_lines" loading="lazy"><span>hough_lines</span></a>
   <a class="opencv-apps-tile" href="nodes/hough_circles.html"><img src="https://ros-perception.github.io/opencv_apps/media/hough_circles_thumb.gif" alt="hough_circles" loading="lazy"><span>hough_circles</span></a>
   <a class="opencv-apps-tile" href="nodes/line_segment_detector.html"><img src="https://ros-perception.github.io/opencv_apps/media/line_segment_detector_thumb.gif" alt="line_segment_detector" loading="lazy"><span>line_segment_detector</span></a>
   <a class="opencv-apps-tile" href="nodes/find_contours.html"><img src="https://ros-perception.github.io/opencv_apps/media/find_contours_thumb.gif" alt="find_contours" loading="lazy"><span>find_contours</span></a>
   <a class="opencv-apps-tile" href="nodes/general_contours.html"><img src="https://ros-perception.github.io/opencv_apps/media/general_contours_thumb.gif" alt="general_contours" loading="lazy"><span>general_contours</span></a>
   <a class="opencv-apps-tile" href="nodes/contour_moments.html"><img src="https://ros-perception.github.io/opencv_apps/media/contour_moments_thumb.gif" alt="contour_moments" loading="lazy"><span>contour_moments</span></a>
   <a class="opencv-apps-tile" href="nodes/convex_hull.html"><img src="https://ros-perception.github.io/opencv_apps/media/convex_hull_thumb.gif" alt="convex_hull" loading="lazy"><span>convex_hull</span></a>
   <a class="opencv-apps-tile" href="nodes/blob_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/blob_detection_thumb.gif" alt="blob_detection" loading="lazy"><span>blob_detection</span></a>
   </div>
   <h3>Segmentation</h3>
   <p>Separate the foreground and the objects: background subtraction, connected components and watershed.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/segment_objects.html"><img src="https://ros-perception.github.io/opencv_apps/media/segment_objects_thumb.gif" alt="segment_objects" loading="lazy"><span>segment_objects</span></a>
   <a class="opencv-apps-tile" href="nodes/watershed_segmentation.html"><img src="https://ros-perception.github.io/opencv_apps/media/watershed_segmentation_thumb.gif" alt="watershed_segmentation" loading="lazy"><span>watershed_segmentation</span></a>
   <a class="opencv-apps-tile" href="nodes/background_subtraction.html"><img src="https://ros-perception.github.io/opencv_apps/media/background_subtraction_thumb.gif" alt="background_subtraction" loading="lazy"><span>background_subtraction</span></a>
   </div>
   <h3>Motion and tracking</h3>
   <p>Estimate the motion with the optical flows and the phase correlation, and track a target given by the mouse or a service.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/fback_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/fback_flow_thumb.gif" alt="fback_flow" loading="lazy"><span>fback_flow</span></a>
   <a class="opencv-apps-tile" href="nodes/lk_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/lk_flow_thumb.gif" alt="lk_flow" loading="lazy"><span>lk_flow</span></a>
   <a class="opencv-apps-tile" href="nodes/simple_flow.html"><img src="https://ros-perception.github.io/opencv_apps/media/simple_flow_thumb.gif" alt="simple_flow" loading="lazy"><span>simple_flow</span></a>
   <a class="opencv-apps-tile" href="nodes/phase_corr.html"><img src="https://ros-perception.github.io/opencv_apps/media/phase_corr_thumb.gif" alt="phase_corr" loading="lazy"><span>phase_corr</span></a>
   <a class="opencv-apps-tile" href="nodes/camshift.html"><img src="https://ros-perception.github.io/opencv_apps/media/camshift_thumb.gif" alt="camshift" loading="lazy"><span>camshift</span></a>
   <a class="opencv-apps-tile" href="nodes/tracking.html"><img src="https://ros-perception.github.io/opencv_apps/media/tracking_thumb.gif" alt="tracking" loading="lazy"><span>tracking</span></a>
   <a class="opencv-apps-tile" href="nodes/match_template.html"><img src="https://ros-perception.github.io/opencv_apps/media/match_template_thumb.gif" alt="match_template" loading="lazy"><span>match_template</span></a>
   </div>
   <h3>Detection and recognition</h3>
   <p>Detect and recognize the faces and the people with the classic detectors, and decode QR codes, barcodes and ArUco markers.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/face_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/face_detection_thumb.gif" alt="face_detection" loading="lazy"><span>face_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/face_recognition.html"><img src="https://ros-perception.github.io/opencv_apps/media/face_recognition_thumb.gif" alt="face_recognition" loading="lazy"><span>face_recognition</span></a>
   <a class="opencv-apps-tile" href="nodes/people_detect.html"><img src="https://ros-perception.github.io/opencv_apps/media/people_detect_thumb.gif" alt="people_detect" loading="lazy"><span>people_detect</span></a>
   <a class="opencv-apps-tile" href="nodes/qrcode_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/qrcode_detection_thumb.gif" alt="qrcode_detection" loading="lazy"><span>qrcode_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/barcode_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/barcode_detection_thumb.gif" alt="barcode_detection" loading="lazy"><span>barcode_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/aruco_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/aruco_detection_thumb.gif" alt="aruco_detection" loading="lazy"><span>aruco_detection</span></a>
   </div>
   <h3>Deep learning (cv::dnn)</h3>
   <p>Run the DNN models of opencv_zoo with cv::dnn on the CPU or CUDA: faces, expressions, objects, image classes, semantic segmentation, depth, text, human and hand poses, and tracking. The models are downloaded at the first run.</p>
   <div class="opencv-apps-gallery">
   <a class="opencv-apps-tile" href="nodes/dnn_face_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_face_detection_thumb.gif" alt="dnn_face_detection" loading="lazy"><span>dnn_face_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_face_recognition.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_face_recognition_thumb.gif" alt="dnn_face_recognition" loading="lazy"><span>dnn_face_recognition</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_facial_expression_recognition.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_facial_expression_recognition_thumb.gif" alt="dnn_facial_expression_recognition" loading="lazy"><span>dnn_facial_expression_recognition</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_object_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_object_detection_thumb.gif" alt="dnn_object_detection" loading="lazy"><span>dnn_object_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_image_classification.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_image_classification_thumb.gif" alt="dnn_image_classification" loading="lazy"><span>dnn_image_classification</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_semantic_segmentation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_semantic_segmentation_thumb.gif" alt="dnn_semantic_segmentation" loading="lazy"><span>dnn_semantic_segmentation</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_depth_estimation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_depth_estimation_thumb.gif" alt="dnn_depth_estimation" loading="lazy"><span>dnn_depth_estimation</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_text_detection.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_text_detection_thumb.gif" alt="dnn_text_detection" loading="lazy"><span>dnn_text_detection</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_pose_estimation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_pose_estimation_thumb.gif" alt="dnn_pose_estimation" loading="lazy"><span>dnn_pose_estimation</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_hand_pose_estimation.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_hand_pose_estimation_thumb.gif" alt="dnn_hand_pose_estimation" loading="lazy"><span>dnn_hand_pose_estimation</span></a>
   <a class="opencv-apps-tile" href="nodes/dnn_tracking.html"><img src="https://ros-perception.github.io/opencv_apps/media/dnn_tracking_thumb.gif" alt="dnn_tracking" loading="lazy"><span>dnn_tracking</span></a>
   </div>
