# Tests simple calibration dataset
catkin_download_test_data(face_detector_withface_test_diamondback.bag http://download.ros.org/data/face_detector/face_detector_withface_test_diamondback.bag
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 59126117e049e69d577b7ee27251a6f8
  )

# sensor_msgs/CameraInfo
catkin_download_test_data(vslam_tutorial_old.bag http://download.ros.org/data/vslam_system/vslam_tutorial.bag
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  FILENAME vslam_tutorial_old.bag
  MD5 f5aece448b7af00a38a993eb71400806
  )
if(sensor_msgs_VERSION VERSION_LESS "1.12.0") # reindex bag file
  catkin_download_test_data(vslam_tutorial.bag http://download.ros.org/data/vslam_system/vslam_tutorial.bag
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
    MD5 f5aece448b7af00a38a993eb71400806
    )
  add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag
    DEPENDS vslam_tutorial_old.bag
    COMMAND rosbag reindex vslam_tutorial_old.bag --output-dir ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial_old.bag ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag
    )
else()  # CameraInfo changes since 1.4.0 so we need fix bag file
  add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag
    DEPENDS vslam_tutorial_old.bag
    COMMAND rosbag fix --force vslam_tutorial_old.bag  ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag
    )
endif()

add_custom_target(vslam_tutorial_bag DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag)
add_dependencies(tests vslam_tutorial_bag)

# test data for face recognition
add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/face_data
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/face_data.tar.gz
  COMMAND ${CMAKE_COMMAND} -E tar zxf face_data.tar.gz
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Extracting face_data.tar.gz")
add_custom_target(face_data DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/face_data)
add_dependencies(tests face_data)

#add_rostest(test-apps.test)
find_package(image_proc)
message(STATUS "check image_proc version (${image_proc_VERSION}), test-adding_images.test check_resize:=true only available since indigo(1.12.0)")
if("${image_proc_VERSION}" VERSION_GREATER "1.12.0") # test-adding_image.test uses image_proc/resize, which only available since indigo
  add_rostest(test-adding_images.test ARGS gui:=false)
endif()
add_rostest(test-discrete_fourier_transform.test ARGS gui:=false)
add_rostest(test-morphology.test ARGS gui:=false)
add_rostest(test-blob-extraction.test ARGS gui:=false)
add_rostest(test-smoothing.test ARGS gui:=false)
add_rostest(test-pyramids.test ARGS gui:=false)
add_rostest(test-threshold.test ARGS gui:=false)
add_rostest(test-rgb_color_filter.test ARGS gui:=false)
add_rostest(test-hls_color_filter.test ARGS gui:=false)
add_rostest(test-hsv_color_filter.test ARGS gui:=false)
add_rostest(test-lab_color_filter.test ARGS gui:=false)
add_rostest(test-edge_detection.test ARGS gui:=false)
add_rostest(test-compare_histogram.test ARGS gui:=false)
add_rostest(test-equalize_histogram.test ARGS gui:=false)
if(OPENCV_HAVE_MATCH_TEMPLATE)
  add_rostest(test-match_template.test ARGS gui:=false)
endif()

add_rostest(test-hough_lines.test ARGS gui:=false)
add_rostest(test-hough_circles.test ARGS gui:=false)
add_rostest(test-blob_detection.test ARGS gui:=false)
add_rostest(test-find_contours.test ARGS gui:=false)
add_rostest(test-convex_hull.test ARGS gui:=false)
add_rostest(test-general_contours.test ARGS gui:=false)
add_rostest(test-contour_moments.test ARGS gui:=false)
if(cv_bridge_VERSION VERSION_LESS "1.11.9") # hydro skip face_detection.test
elseif("$ENV{ROS_DISTRO}" STREQUAL "noetic") # noetic uses opencv4 (need to use_opencv3 for some reason...)
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv4:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv4:=true)
elseif("$ENV{ROS_DISTRO}" STREQUAL "melodic") # melodic uses opencv3
  add_rostest(test-face_detection.test ARGS gui:=false)
  add_rostest(test-face_recognition.test ARGS gui:=false)
elseif(OpenCV_VERSION VERSION_LESS "3.0")
  add_rostest(test-face_detection.test ARGS gui:=false)
  add_rostest(test-face_recognition.test ARGS gui:=false)
elseif(OpenCV_VERSION VERSION_LESS "3.2")
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_1:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_1:=true)
elseif(OpenCV_VERSION VERSION_LESS "3.3")
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_2:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_2:=true)
elseif(OpenCV_VERSION VERSION_LESS "3.4")
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_3:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_3:=true)
else()
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv4:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv4:=true)
endif()
add_rostest(test-goodfeature_track.test ARGS gui:=false)
add_rostest(test-corner_harris.test ARGS gui:=false)
add_rostest(test-camshift.test ARGS gui:=false)
add_rostest(test-fback_flow.test ARGS gui:=false)
add_rostest(test-lk_flow.test ARGS gui:=false)
add_rostest(test-people_detect.test ARGS gui:=false)
add_rostest(test-phase_corr.test ARGS gui:=false)
add_rostest(test-segment_objects.test ARGS gui:=false)
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  add_rostest(test-simple_flow.test ARGS gui:=false)
endif()
if(OPENCV_HAVE_LSD)
  add_rostest(test-line_segment_detector.test ARGS gui:=false)
endif()
add_rostest(test-watershed_segmentation.test ARGS gui:=false)
if(OPENCV_HAVE_BACKGROUND_SUBTRACTION)
  add_rostest(test-background_subtraction.test ARGS gui:=false)
endif()
# tracking requires OpenCV 4.5.1 or later, and KCF tracker is in opencv_contrib tracking module
if(OPENCV_HAVE_TRACKING_API AND TARGET opencv_tracking)
  add_rostest(test-tracking.test ARGS gui:=false)
elseif(OPENCV_HAVE_TRACKING_API)
  add_rostest(test-tracking.test ARGS gui:=false tracking_algorithm:=0)
endif()

add_rostest(test-simple_example.test ARGS gui:=false)
add_rostest(test-simple_compressed_example.test ARGS gui:=false)

# OpenCV 4 nodes, see OPENCV_HAVE_DNN in ../CMakeLists.txt
set(_skip_test_files)
if(NOT OPENCV_HAVE_LSD)
  list(APPEND _skip_test_files test-line_segment_detector.test)
endif()
# the tests of the dnn nodes download the models by libcurl
if(OPENCV_HAVE_DNN AND CURL_FOUND)
  add_rostest(test-dnn_face_detection.test ARGS gui:=false)
  add_rostest(test-dnn_face_recognition.test ARGS gui:=false)
  add_rostest(test-dnn_object_detection.test ARGS gui:=false)
  add_rostest(test-dnn_text_detection.test ARGS gui:=false)
  add_rostest(test-dnn_semantic_segmentation.test ARGS gui:=false)
  add_rostest(test-dnn_depth_estimation.test ARGS gui:=false)
  add_rostest(test-dnn_image_classification.test ARGS gui:=false)
  add_rostest(test-dnn_facial_expression_recognition.test ARGS gui:=false)
else()
  list(APPEND _skip_test_files test-dnn_face_detection.test test-dnn_face_recognition.test
    test-dnn_object_detection.test test-dnn_text_detection.test
    test-dnn_semantic_segmentation.test test-dnn_depth_estimation.test
    test-dnn_image_classification.test test-dnn_facial_expression_recognition.test)
endif()
# the MediaPipe models of dnn_pose_estimation and dnn_hand_pose_estimation need OpenCV 4.9 or later
if(OPENCV_HAVE_DNN AND CURL_FOUND AND NOT OpenCV_VERSION VERSION_LESS "4.9")
  add_rostest(test-dnn_pose_estimation.test ARGS gui:=false)
  add_rostest(test-dnn_hand_pose_estimation.test ARGS gui:=false)
else()
  list(APPEND _skip_test_files test-dnn_pose_estimation.test test-dnn_hand_pose_estimation.test)
endif()
# detection_3d, built with tf2_ros and vision_msgs; the test detects the people with dnn_object_detection
if(OPENCV_APPS_HAVE_DETECTION_3D AND OPENCV_HAVE_DNN AND CURL_FOUND)
  add_rostest(test-detection_3d.test ARGS gui:=false)
  add_rostest(test-detection_3d_points.test ARGS gui:=false)
else()
  list(APPEND _skip_test_files test-detection_3d.test test-detection_3d_points.test)
endif()
if(OPENCV_HAVE_DNN)
  add_rostest(test-qrcode_detection.test ARGS gui:=false)
else()
  list(APPEND _skip_test_files test-qrcode_detection.test)
endif()
if(OPENCV_HAVE_BARCODE)
  add_rostest(test-barcode_detection.test ARGS gui:=false)
else()
  list(APPEND _skip_test_files test-barcode_detection.test)
endif()
if(OPENCV_HAVE_ARUCO)
  add_rostest(test-aruco_detection.test ARGS gui:=false)
else()
  list(APPEND _skip_test_files test-aruco_detection.test)
endif()

if(roslaunch_VERSION VERSION_LESS "1.11.1")
  message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
else()
  file(GLOB LAUNCH_FILES *.test)
  foreach(_skip_test_file ${_skip_test_files})
    list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${_skip_test_file})
  endforeach()
  if(NOT OPENCV_HAVE_BACKGROUND_SUBTRACTION)
    list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test-background_subtraction.test)
  endif()
  if(NOT OPENCV_HAVE_MATCH_TEMPLATE)
    list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test-match_template.test)
  endif()
  if(NOT OPENCV_HAVE_TRACKING_API)
    list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/test-tracking.test)
  endif()
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()
