# Build optimized binaries by default for source builds (catkin_make,
# catkin build, colcon) when no build type is given. Package builds
# (bloom/debhelper) pass their own CMAKE_BUILD_TYPE, and multi-config
# generators ignore it. Called after project(), so that a build type set by
# the toolchain file is respected.
macro(opencv_apps_default_build_type)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    #catkin_lint: ignore_once global_var_collision
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE)
  endif()
endmacro()

# $ENV{ROS_VERSION} is set by ros_environment, which is missing on
# pre-ros_environment distros (e.g. indigo) and on ROS2 builds that do not
# depend on it (e.g. the buildfarm's binary jobs). Guess it then: ament sets
# AMENT_PREFIX_PATH (bloom's debian/rules passes it to cmake, setup.sh and
# colcon set the environment variable), catkin never does.
if("$ENV{ROS_VERSION}" STREQUAL "")
  #catkin_lint: skip
  if(DEFINED AMENT_PREFIX_PATH OR DEFINED ENV{AMENT_PREFIX_PATH})
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
  message(WARNING "ROS_VERSION is not set, assuming ROS_VERSION=$ENV{ROS_VERSION}")
endif()

if($ENV{ROS_VERSION} EQUAL 2)
#catkin_lint: skip (catkin_lint does not evaluate $ENV{ROS_VERSION}, and checks only the ROS1 part below)
cmake_minimum_required(VERSION 3.8)
project(opencv_apps)
opencv_apps_default_build_type()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_definitions(-DROS2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
if(TARGET opencv_optflow)
  set(OPENCV_HAVE_OPTFLOW TRUE)
endif()
# cv::createLineSegmentDetector() is not in OpenCV 2.x, and throws "Implementation has been removed due original code
# license issues" on OpenCV 3.4.6 - 4.5.3 (e.g. noetic's 4.2), it is back since 4.5.4
if(NOT OpenCV_VERSION VERSION_LESS "3.0" AND
    (OpenCV_VERSION VERSION_LESS "3.4.6" OR NOT OpenCV_VERSION VERSION_LESS "4.5.4"))
  set(OPENCV_HAVE_LSD TRUE)
endif()
# dnn and objdetect nodes require OpenCV 4.5.4 (cv::FaceDetectorYN) or later, e.g. Ubuntu 24.04
if(NOT OpenCV_VERSION VERSION_LESS "4.5.4")
  set(OPENCV_HAVE_DNN TRUE)
  # libcurl downloads the models on the first run, without it the models must be given as files
  find_package(CURL)
  if(CURL_FOUND)
    add_definitions(-DOPENCV_APPS_HAVE_CURL)
  else()
    message(WARNING "libcurl is not found, the dnn nodes do not download the models")
  endif()
  find_package(geometry_msgs REQUIRED)
  # barcode is in objdetect since OpenCV 4.8, and aruco since 4.7, they were in opencv-contrib
  if(NOT OpenCV_VERSION VERSION_LESS "4.8" OR TARGET opencv_barcode)
    set(OPENCV_HAVE_BARCODE TRUE)
  endif()
  if(NOT OpenCV_VERSION VERSION_LESS "4.7" OR TARGET opencv_aruco)
    set(OPENCV_HAVE_ARUCO TRUE)
  endif()
endif()
# MOG, GMG, CNT, GSOC and LSBP of background_subtraction are in opencv_contrib's bgsegm module
if(TARGET opencv_bgsegm)
  set(OPENCV_HAVE_BGSEGM TRUE)
endif()

file(GLOB _msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg srv/*.srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${_msg_files}
  DEPENDENCIES sensor_msgs std_msgs
)
rosidl_get_typesupport_target(_cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# nodelets are rclcpp components, and each has an executable with the same name as ROS1
set(_nodelets
  adding_images:AddingImagesNodelet
  compare_histogram:CompareHistogramNodelet
  discrete_fourier_transform:DiscreteFourierTransformNodelet
  equalize_histogram:EqualizeHistogramNodelet
  match_template:MatchTemplateNodelet
  morphology:MorphologyNodelet
  pyramids:PyramidsNodelet
  smoothing:SmoothingNodelet
  threshold:ThresholdNodelet
  rgb_color_filter:RGBColorFilterNodelet
  hls_color_filter:HLSColorFilterNodelet
  hsv_color_filter:HSVColorFilterNodelet
  lab_color_filter:LabColorFilterNodelet
  edge_detection:EdgeDetectionNodelet
  hough_lines:HoughLinesNodelet
  hough_circles:HoughCirclesNodelet
  blob_detection:BlobDetectionNodelet
  find_contours:FindContoursNodelet
  convex_hull:ConvexHullNodelet
  general_contours:GeneralContoursNodelet
  contour_moments:ContourMomentsNodelet
  goodfeature_track:GoodfeatureTrackNodelet
  corner_harris:CornerHarrisNodelet
  camshift:CamShiftNodelet
  face_detection:FaceDetectionNodelet
  face_recognition:FaceRecognitionNodelet
  fback_flow:FBackFlowNodelet
  lk_flow:LKFlowNodelet
  people_detect:PeopleDetectNodelet
  phase_corr:PhaseCorrNodelet
  segment_objects:SegmentObjectsNodelet
  watershed_segmentation:WatershedSegmentationNodelet
  simple_example:SimpleExampleNodelet
  simple_compressed_example:SimpleCompressedExampleNodelet
  background_subtraction:BackgroundSubtractionNodelet
)
set(_nodelet_cppfiles
  src/nodelet/adding_images_nodelet.cpp
  src/nodelet/compare_histogram_nodelet.cpp
  src/nodelet/discrete_fourier_transform_nodelet.cpp
  src/nodelet/equalize_histogram_nodelet.cpp
  src/nodelet/match_template_nodelet.cpp
  src/nodelet/morphology_nodelet.cpp
  src/nodelet/pyramids_nodelet.cpp
  src/nodelet/smoothing_nodelet.cpp
  src/nodelet/threshold_nodelet.cpp
  src/nodelet/color_filter_nodelet.cpp
  src/nodelet/edge_detection_nodelet.cpp
  src/nodelet/hough_lines_nodelet.cpp
  src/nodelet/hough_circles_nodelet.cpp
  src/nodelet/blob_detection_nodelet.cpp
  src/nodelet/find_contours_nodelet.cpp
  src/nodelet/convex_hull_nodelet.cpp
  src/nodelet/general_contours_nodelet.cpp
  src/nodelet/contour_moments_nodelet.cpp
  src/nodelet/goodfeature_track_nodelet.cpp
  src/nodelet/corner_harris_nodelet.cpp
  src/nodelet/camshift_nodelet.cpp
  src/nodelet/face_detection_nodelet.cpp
  src/nodelet/face_recognition_nodelet.cpp
  src/nodelet/fback_flow_nodelet.cpp
  src/nodelet/lk_flow_nodelet.cpp
  src/nodelet/people_detect_nodelet.cpp
  src/nodelet/phase_corr_nodelet.cpp
  src/nodelet/segment_objects_nodelet.cpp
  src/nodelet/watershed_segmentation_nodelet.cpp
  src/nodelet/simple_example_nodelet.cpp
  src/nodelet/simple_compressed_example_nodelet.cpp
  src/nodelet/background_subtraction_nodelet.cpp
)
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  list(APPEND _nodelets simple_flow:SimpleFlowNodelet)
  list(APPEND _nodelet_cppfiles src/nodelet/simple_flow_nodelet.cpp)
endif()
if(OPENCV_HAVE_LSD)
  list(APPEND _nodelets line_segment_detector:LineSegmentDetectorNodelet)
  list(APPEND _nodelet_cppfiles src/nodelet/line_segment_detector_nodelet.cpp)
endif()
if(OPENCV_HAVE_DNN)
  list(APPEND _nodelets
    dnn_face_detection:DNNFaceDetectionNodelet
    dnn_face_recognition:DNNFaceRecognitionNodelet
    dnn_object_detection:DNNObjectDetectionNodelet
    dnn_tracking:DNNTrackingNodelet
    dnn_facial_expression_recognition:DNNFacialExpressionRecognitionNodelet
    dnn_image_classification:DNNImageClassificationNodelet
    dnn_depth_estimation:DNNDepthEstimationNodelet
    dnn_pose_estimation:DNNPoseEstimationNodelet
    dnn_hand_pose_estimation:DNNHandPoseEstimationNodelet
    dnn_semantic_segmentation:DNNSemanticSegmentationNodelet
    dnn_text_detection:DNNTextDetectionNodelet
    qrcode_detection:QRCodeDetectionNodelet
  )
  list(APPEND _nodelet_cppfiles
    src/nodelet/dnn.cpp
    src/nodelet/dnn_face_detection_nodelet.cpp
    src/nodelet/dnn_face_recognition_nodelet.cpp
    src/nodelet/dnn_object_detection_nodelet.cpp
    src/nodelet/dnn_tracking_nodelet.cpp
    src/nodelet/dnn_facial_expression_recognition_nodelet.cpp
    src/nodelet/dnn_image_classification_nodelet.cpp
    src/nodelet/dnn_depth_estimation_nodelet.cpp
    src/nodelet/dnn_pose_estimation_nodelet.cpp
    src/nodelet/dnn_hand_pose_estimation_nodelet.cpp
    src/nodelet/dnn_semantic_segmentation_nodelet.cpp
    src/nodelet/dnn_text_detection_nodelet.cpp
    src/nodelet/qrcode_detection_nodelet.cpp
  )
  if(OPENCV_HAVE_BARCODE)
    list(APPEND _nodelets barcode_detection:BarcodeDetectionNodelet)
    list(APPEND _nodelet_cppfiles src/nodelet/barcode_detection_nodelet.cpp)
  endif()
  if(OPENCV_HAVE_ARUCO)
    list(APPEND _nodelets aruco_detection:ArucoDetectionNodelet)
    list(APPEND _nodelet_cppfiles src/nodelet/aruco_detection_nodelet.cpp)
  endif()
endif()
if(OPENCV_HAVE_BGSEGM)
  set_source_files_properties(src/nodelet/background_subtraction_nodelet.cpp PROPERTIES
    COMPILE_DEFINITIONS OPENCV_APPS_HAVE_BGSEGM)
endif()
# tracking requires the tracking API of OpenCV 4.5.1 or later, and KCF, CSRT and the legacy trackers are in
# opencv_contrib tracking module
if(OpenCV_VERSION VERSION_GREATER_EQUAL "4.5.1")
  set(OPENCV_HAVE_TRACKING_API TRUE)
  list(APPEND _nodelets tracking:TrackingNodelet)
  list(APPEND _nodelet_cppfiles src/nodelet/tracking_nodelet.cpp)
  if(TARGET opencv_tracking)
    set_source_files_properties(src/nodelet/tracking_nodelet.cpp PROPERTIES COMPILE_DEFINITIONS OPENCV_APPS_HAVE_TRACKING)
  endif()
endif()
# detection_3d: the 3D boxes (vision_msgs) and the TF frames of the 2D detections, with a depth image
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)
list(APPEND _nodelets detection_3d:Detection3DNodelet)
list(APPEND _nodelet_cppfiles src/nodelet/detection_3d_nodelet.cpp)

add_library(${PROJECT_NAME}_nodelet SHARED
  src/nodelet/nodelet.cpp
  ${_nodelet_cppfiles}
)
target_include_directories(${PROJECT_NAME}_nodelet PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${_cpp_typesupport_target}
  cv_bridge::cv_bridge
  image_transport::image_transport
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${OpenCV_LIBRARIES}
)
if(OPENCV_HAVE_DNN)
  target_link_libraries(${PROJECT_NAME}_nodelet ${geometry_msgs_TARGETS})
  if(CURL_FOUND)
    target_link_libraries(${PROJECT_NAME}_nodelet CURL::libcurl)
  endif()
endif()
target_link_libraries(${PROJECT_NAME}_nodelet ${geometry_msgs_TARGETS} tf2_ros::tf2_ros ${vision_msgs_TARGETS})
foreach(_nodelet ${_nodelets})
  string(REPLACE ":" ";" _nodelet ${_nodelet})
  list(GET _nodelet 0 _name)
  list(GET _nodelet 1 _class)
  rclcpp_components_register_node(${PROJECT_NAME}_nodelet
    PLUGIN "opencv_apps::${_class}"
    EXECUTABLE ${_name})
endforeach()

install(TARGETS ${PROJECT_NAME}_nodelet
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.xml"
)
# with a python3 shebang (the scripts keep "python" for ROS1 melodic and older)
foreach(_script face_recognition_trainer.py set_image.py)
  file(READ scripts/${_script} _script_content)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS scripts/${_script})
  string(REGEX REPLACE "^#! */usr/bin/env python\n" "#!/usr/bin/env python3\n" _script_content "${_script_content}")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${_script} "${_script_content}")
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${_script}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  # test/ros2/test_*.py: ROS2 versions of test/test-*.test, plus test_nodes.py; they import test/ros2/opencv_apps_test.py
  file(GLOB _launch_tests RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} test/ros2/test_*.py)
  if(OPENCV_HAVE_OPTFLOW)
    set(_have_optflow 1)
  else()
    set(_have_optflow 0)
    list(REMOVE_ITEM _launch_tests test/ros2/test_simple_flow.py)
  endif()
  # the tests of the dnn nodes download the models by libcurl
  if(NOT OPENCV_HAVE_DNN OR NOT CURL_FOUND)
    list(REMOVE_ITEM _launch_tests test/ros2/test_dnn_face_detection.py test/ros2/test_dnn_face_recognition.py
      test/ros2/test_dnn_object_detection.py test/ros2/test_dnn_text_detection.py
      test/ros2/test_dnn_semantic_segmentation.py test/ros2/test_dnn_depth_estimation.py
      test/ros2/test_dnn_image_classification.py test/ros2/test_dnn_facial_expression_recognition.py)
  endif()
  # the test of detection_3d detects the people with dnn_object_detection
  if(NOT OPENCV_HAVE_DNN OR NOT CURL_FOUND)
    list(REMOVE_ITEM _launch_tests test/ros2/test_detection_3d.py test/ros2/test_detection_3d_points.py)
  endif()
  if(NOT OPENCV_HAVE_DNN)
    list(REMOVE_ITEM _launch_tests test/ros2/test_qrcode_detection.py)
  endif()
  # the tests of dnn_tracking use NanoTrack (OpenCV 4.7 or later) and VitTrack (OpenCV 4.9 or later), whose models
  # are small, not DaSiamRPN, whose models are 160 MB
  if(NOT OPENCV_HAVE_DNN OR NOT CURL_FOUND OR OpenCV_VERSION VERSION_LESS "4.7")
    list(REMOVE_ITEM _launch_tests test/ros2/test_dnn_tracking.py test/ros2/test_dnn_tracking_nanotrack.py
      test/ros2/test_dnn_tracking_walk.py)
  endif()
  # the MediaPipe models of dnn_pose_estimation and dnn_hand_pose_estimation need OpenCV 4.9 or later
  if(NOT OPENCV_HAVE_DNN OR NOT CURL_FOUND OR OpenCV_VERSION VERSION_LESS "4.9")
    list(REMOVE_ITEM _launch_tests test/ros2/test_dnn_pose_estimation.py test/ros2/test_dnn_pose_estimation_tracking.py
      test/ros2/test_dnn_hand_pose_estimation.py test/ros2/test_dnn_hand_pose_estimation_tracking.py)
  endif()
  if(NOT OPENCV_HAVE_BARCODE)
    list(REMOVE_ITEM _launch_tests test/ros2/test_barcode_detection.py)
  endif()
  if(NOT OPENCV_HAVE_ARUCO)
    list(REMOVE_ITEM _launch_tests test/ros2/test_aruco_detection.py)
  endif()
  if(OPENCV_HAVE_LSD)
    set(_have_lsd 1)
  else()
    set(_have_lsd 0)
    list(REMOVE_ITEM _launch_tests test/ros2/test_line_segment_detector.py)
  endif()
  if(NOT OPENCV_HAVE_TRACKING_API)
    list(REMOVE_ITEM _launch_tests test/ros2/test_tracking.py test/ros2/test_tracking_walk.py)
  endif()
  if(TARGET opencv_tracking)
    set(_have_tracking 1)
  else()
    set(_have_tracking 0)
    # CSRT tracker is in opencv_contrib tracking module
    list(REMOVE_ITEM _launch_tests test/ros2/test_tracking_walk.py)
  endif()
  foreach(_launch_test ${_launch_tests})
    get_filename_component(_name ${_launch_test} NAME_WE)
    add_launch_test(${_launch_test} TARGET ${_name} TIMEOUT 180
      ENV OPENCV_APPS_HAVE_OPTFLOW=${_have_optflow} OPENCV_APPS_HAVE_LSD=${_have_lsd}
          OPENCV_APPS_HAVE_TRACKING=${_have_tracking}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/ros2)
    # they use the same ROS domain and topic names (e.g. /image)
    set_tests_properties(${_name} PROPERTIES RUN_SERIAL TRUE)
  endforeach()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

else() # if($ENV{ROS_VERSION} EQUAL 2)
#catkin_lint: ignore_once order_violation (only the ROS_VERSION guess above comes before it)
cmake_minimum_required(VERSION 3.5)
project(opencv_apps)
opencv_apps_default_build_type()

## https://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7")
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "C++11 activated.")
    add_definitions("-std=gnu++11")
  elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
    message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
    add_definitions("-std=gnu++0x")
  else ()
    message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
  endif()
endif()

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure geometry_msgs message_generation image_transport nodelet roscpp sensor_msgs std_msgs std_srvs class_loader)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV Components: ${OpenCV_LIB_COMPONENTS}")
if(OpenCV_VERSION VERSION_LESS "3.0" OR TARGET opencv_optflow)
  set(OPENCV_HAVE_OPTFLOW TRUE)
endif()
# dnn and objdetect nodes require OpenCV 4.5.4 (cv::FaceDetectorYN) or later, e.g. Ubuntu 24.04,
# and C++17 (std::filesystem), which is the default since GCC 11. The C++ standard is not raised here
# not to change the other nodes, e.g. on focal (GCC 9, C++14) with OpenCV 4.5.4 of JetPack 5
# the standard is CMAKE_CXX_STANDARD, -std= of CMAKE_CXX_FLAGS, or the default of the compiler, in this order
set(_opencv_apps_cxx_standard ${CMAKE_CXX_STANDARD})
if(NOT _opencv_apps_cxx_standard AND CMAKE_CXX_FLAGS MATCHES "-std=(c|gnu)\\+\\+([0-9a-z]+)")
  set(_opencv_apps_cxx_standard ${CMAKE_MATCH_2})
endif()
if(NOT _opencv_apps_cxx_standard)
  set(_opencv_apps_cxx_standard ${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT})
endif()
if(NOT _opencv_apps_cxx_standard OR _opencv_apps_cxx_standard MATCHES "^(98|03|0x|11|1y|14)$"
    OR (CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9"))  # std::filesystem of GCC 9
  if(NOT OpenCV_VERSION VERSION_LESS "4.5.4")
    message(WARNING "The dnn and objdetect nodes are not built, as they require C++17 (C++${_opencv_apps_cxx_standard} is used, set -DCMAKE_CXX_STANDARD=17 to build them) and std::filesystem of GCC 9 or later")
  endif()
elseif(NOT OpenCV_VERSION VERSION_LESS "4.5.4")
  set(OPENCV_HAVE_DNN TRUE)
  # libcurl downloads the models on the first run, without it the models must be given as files
  find_package(CURL)
  if(CURL_FOUND)
    add_definitions(-DOPENCV_APPS_HAVE_CURL)
  else()
    message(WARNING "libcurl is not found, the dnn nodes do not download the models")
  endif()
  # barcode is in objdetect since OpenCV 4.8, and aruco since 4.7, they were in opencv-contrib
  if(NOT OpenCV_VERSION VERSION_LESS "4.8" OR TARGET opencv_barcode)
    set(OPENCV_HAVE_BARCODE TRUE)
  endif()
  if(NOT OpenCV_VERSION VERSION_LESS "4.7" OR TARGET opencv_aruco)
    set(OPENCV_HAVE_ARUCO TRUE)
  endif()
endif()
# MOG, GMG, CNT, GSOC and LSBP of background_subtraction are in opencv_contrib's bgsegm module
if(TARGET opencv_bgsegm)
  set(OPENCV_HAVE_BGSEGM TRUE)
endif()
# cv::createBackgroundSubtractorMOG2() and the cv::BackgroundSubtractor interface of background_subtraction are
# since OpenCV 3.0
if(NOT OpenCV_VERSION VERSION_LESS "3.0")
  set(OPENCV_HAVE_BACKGROUND_SUBTRACTION TRUE)
endif()
# match_template uses the mask of cv::matchTemplate() and cv::IMREAD_* of OpenCV 3.0 or later
if(NOT OpenCV_VERSION VERSION_LESS "3.0")
  set(OPENCV_HAVE_MATCH_TEMPLATE TRUE)
endif()
# cv::createLineSegmentDetector() is not in OpenCV 2.x, and throws "Implementation has been removed due original code
# license issues" on OpenCV 3.4.6 - 4.5.3 (e.g. noetic's 4.2), it is back since 4.5.4
if(NOT OpenCV_VERSION VERSION_LESS "3.0" AND
    (OpenCV_VERSION VERSION_LESS "3.4.6" OR NOT OpenCV_VERSION VERSION_LESS "4.5.4"))
  set(OPENCV_HAVE_LSD TRUE)
endif()
# Supporting CompressedImage in cv_bridge has been started from 1.11.9 (2015-11-29)
# note : hydro 1.10.18, indigo : 1.11.13 ...
# https://github.com/ros-perception/vision_opencv/pull/70
if(cv_bridge_VERSION VERSION_LESS "1.11.9")
  add_definitions("-DCV_BRIDGE_COMPRESSED_IMAGE_IS_NOT_SUPPORTED")
endif()
# Supporting CvtColorForDisplay in cv_bridge has been started from 1.11.9 (2015-11-29)
if(cv_bridge_VERSION VERSION_LESS "1.11.9")
  add_definitions("-DCV_BRIDGE_CVT_COLOR_FOR_DISPLAY_IS_NOT_SUPPORTED")
endif()
# Supporting CvtColorForDisplayOptions in cv_bridge has been started from 1.11.13 (2016-07-11)
if(cv_bridge_VERSION VERSION_LESS "1.11.13")
  add_definitions("-DCV_BRIDGE_CVT_COLOR_FOR_DISPLAY_OPTION_IS_NOT_SUPPORTED")
endif()
if(class_loader_VERSION VERSION_LESS "0.4.0")
  add_definitions("-DUSE_PLUGINLIB_CLASS_LIST_MACROS_H")
endif()

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/DiscreteFourierTransform.cfg
  cfg/AddingImages.cfg
  cfg/Smoothing.cfg
  cfg/Morphology.cfg
  cfg/Pyramids.cfg
  cfg/EdgeDetection.cfg cfg/HoughLines.cfg cfg/HoughCircles.cfg
  cfg/FindContours.cfg cfg/ConvexHull.cfg cfg/GeneralContours.cfg cfg/ContourMoments.cfg
  cfg/FaceDetection.cfg
  cfg/FaceRecognition.cfg
  cfg/GoodfeatureTrack.cfg
  cfg/CornerHarris.cfg
  #
  cfg/CompareHistogram.cfg
  cfg/EqualizeHistogram.cfg
  cfg/CamShift.cfg
  cfg/FBackFlow.cfg
  cfg/LKFlow.cfg
  cfg/MatchTemplate.cfg
  cfg/PeopleDetect.cfg
  cfg/PhaseCorr.cfg
  cfg/SegmentObjects.cfg
  cfg/SimpleFlow.cfg
  cfg/Threshold.cfg
  cfg/RGBColorFilter.cfg
  cfg/HLSColorFilter.cfg
  cfg/HSVColorFilter.cfg
  cfg/LabColorFilter.cfg
  cfg/LineSegmentDetector.cfg
  cfg/WatershedSegmentation.cfg
  cfg/BlobDetection.cfg
  cfg/BackgroundSubtraction.cfg
  cfg/Tracking.cfg
  )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Point2D.msg
   Point2DStamped.msg
   Point2DArray.msg
   Point2DArrayStamped.msg
   Rect.msg
   RectArray.msg
   RectArrayStamped.msg
   Flow.msg
   FlowStamped.msg
   FlowArray.msg
   FlowArrayStamped.msg
   Size.msg
   Face.msg
   FaceArray.msg
   FaceArrayStamped.msg
   Line.msg
   LineArray.msg
   LineArrayStamped.msg
   RotatedRect.msg
   RotatedRectStamped.msg
   RotatedRectArray.msg
   RotatedRectArrayStamped.msg
   Circle.msg
   CircleArray.msg
   CircleArrayStamped.msg
   Moment.msg
   MomentArray.msg
   MomentArrayStamped.msg
   Contour.msg
   ContourArray.msg
   ContourArrayStamped.msg
   Detection.msg
   DetectionArray.msg
   DetectionArrayStamped.msg
   Keypoint.msg
   Skeleton.msg
   SkeletonArray.msg
   SkeletonArrayStamped.msg
)

add_service_files(
  FILES
  FaceRecognitionTrain.srv
  SetROI.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure message_runtime nodelet roscpp sensor_msgs std_msgs std_srvs
#                DEPENDS OpenCV
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

# rclcpp/rclcpp.hpp, <pkg>/msg/foo.hpp, ... for the nodes written for ROS2, see ros1/include/opencv_apps/ros1_compat.h
include_directories(ros1/include)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Macro to add nodelets
macro(opencv_apps_add_nodelet node_name nodelet_cppfile)
  set(NODE_NAME ${node_name})
  set(NODELET_NAME opencv_apps/${node_name})
  configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
  add_executable(${node_name}_exe ${node_name}.cpp)
  SET_TARGET_PROPERTIES(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
  target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  list(APPEND _opencv_apps_nodelet_cppfiles ${nodelet_cppfile})
  list(APPEND _opencv_apps_nodelet_targets ${node_name}_exe)
endmacro()

# https://github.com/Itseez/opencv/blob/2.4/samples/cpp/

# calib3d
  # ./tutorial_code/calib3d/camera_calibration/camera_calibration.cpp
  # ./tutorial_code/calib3d/real_time_pose_estimation/src/main_detection.cpp
  # ./tutorial_code/calib3d/real_time_pose_estimation/src/main_registration.cpp
  # ./tutorial_code/calib3d/stereoBM/SBM_Sample.cpp

# core
opencv_apps_add_nodelet(adding_images src/nodelet/adding_images_nodelet.cpp) # ./tutorial_code/core/AddingImages/AddingImages.cpp
opencv_apps_add_nodelet(discrete_fourier_transform src/nodelet/discrete_fourier_transform_nodelet.cpp)  # ./tutorial_code/core/discrete_fourier_transform/discrete_fourier_transform.cpp
  # ./tutorial_code/core/file_input_output/file_input_output.cpp
  # ./tutorial_code/core/how_to_scan_images/how_to_scan_images.cpp
  # ./tutorial_code/core/interoperability_with_OpenCV_1/interoperability_with_OpenCV_1.cpp
  # ./tutorial_code/core/ippasync/ippasync_sample.cpp
  # ./tutorial_code/core/mat_mask_operations/mat_mask_operations.cpp
  # ./tutorial_code/core/Matrix/Drawing_1.cpp
  # ./tutorial_code/core/Matrix/Drawing_2.cpp
  # ./tutorial_code/core/mat_the_basic_image_container/mat_the_basic_image_container.cpp

# features2D
  # ./tutorial_code/features2D/AKAZE_match.cpp
  # ./tutorial_code/features2D/AKAZE_tracking/planar_tracking.cpp
  # ./tutorial_code/gpu/gpu-basics-similarity/gpu-basics-similarity.cpp

# highGUi
  # ./tutorial_code/HighGUI/AddingImagesTrackbar.cpp
  # ./tutorial_code/HighGUI/BasicLinearTransformsTrackbar.cpp

# Histograms_Matching
  # ./tutorial_code/Histograms_Matching/calcBackProject_Demo1.cpp
  # ./tutorial_code/Histograms_Matching/calcBackProject_Demo2.cpp
  # ./tutorial_code/Histograms_Matching/calcHist_Demo.cpp
  #  ./tutorial_code/Histograms_Matching/compareHist_Demo.cpp
  opencv_apps_add_nodelet(compare_histogram src/nodelet/compare_histogram_nodelet.cpp) # calcHist_Demo.cpp and compareHist_Demo.cpp
  opencv_apps_add_nodelet(equalize_histogram src/nodelet/equalize_histogram_nodelet.cpp) # ./tutorial_code/Histograms_Matching/EqualizeHist_Demo.cpp
  # ./tutorial_code/Histograms_Matching/MatchTemplate_Demo.cpp
if(OPENCV_HAVE_MATCH_TEMPLATE)
  opencv_apps_add_nodelet(match_template src/nodelet/match_template_nodelet.cpp) # ./tutorial_code/Histograms_Matching/MatchTemplate_Demo.cpp
endif()

# imagecodecs
  # ./tutorial_code/imgcodecs/GDAL_IO/gdal-image.cpp

# ImgProc
  # ./tutorial_code/ImgProc/BasicLinearTransforms.cpp
  # ./tutorial_code/ImgProc/Morphology_1.cpp
  # ./tutorial_code/ImgProc/Morphology_2.cpp
opencv_apps_add_nodelet(morphology src/nodelet/morphology_nodelet.cpp)  # ./tutorial_code/ImgProc/Morphology_3.cpp
opencv_apps_add_nodelet(pyramids src/nodelet/pyramids_nodelet.cpp)  # ./tutorial_code/ImgProc/Pyramids.cpp
opencv_apps_add_nodelet(smoothing src/nodelet/smoothing_nodelet.cpp)  # ./tutorial_code/ImgProc/Smoothing.cpp
opencv_apps_add_nodelet(threshold src/nodelet/threshold_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold.cpp
opencv_apps_add_nodelet(rgb_color_filter src/nodelet/color_filter_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold_inRange.cpp
opencv_apps_add_nodelet(hls_color_filter src/nodelet/color_filter_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold_inRange.cpp
opencv_apps_add_nodelet(hsv_color_filter src/nodelet/color_filter_nodelet.cpp)  # ./tutorial_code/ImgProc/Threshold_inRange.cpp
opencv_apps_add_nodelet(lab_color_filter src/nodelet/color_filter_nodelet.cpp)

# ImgTrans
opencv_apps_add_nodelet(edge_detection src/nodelet/edge_detection_nodelet.cpp) # ./tutorial_code/ImgTrans/CannyDetector_Demo.cpp
                                                                                                             # ./tutorial_code/ImgTrans/Laplace_Demo.cpp
                                                                                                             # ./tutorial_code/ImgTrans/Sobel_Demo.cpp
opencv_apps_add_nodelet(hough_lines src/nodelet/hough_lines_nodelet.cpp) # ./tutorial_code/ImgTrans/HoughLines_Demo.cpp
opencv_apps_add_nodelet(hough_circles src/nodelet/hough_circles_nodelet.cpp) # ./tutorial_code/ImgTrans/HoughCircle_Demo.cpp
 # ./tutorial_code/ImgTrans/copyMakeBorder_demo.cpp
 # ./tutorial_code/ImgTrans/filter2D_demo.cpp
 # ./tutorial_code/ImgTrans/Geometric_Transforms_Demo.cpp
 # ./tutorial_code/ImgTrans/imageSegmentation.cpp
 # ./tutorial_code/ImgTrans/Remap_Demo.cpp

# introduction
 # ./tutorial_code/introduction/display_image/display_image.cpp
 # ./tutorial_code/introduction/windows_visual_studio_Opencv/introduction_windows_vs.cpp

# ml
 # ./tutorial_code/ml/introduction_to_pca/introduction_to_pca.cpp
 # ./tutorial_code/ml/introduction_to_svm/introduction_to_svm.cpp
 # ./tutorial_code/ml/non_linear_svms/non_linear_svms.cpp

# objectDetection
 # ./tutorial_code/objectDetection/objectDetection2.cpp
 # ./tutorial_code/objectDetection/objectDetection.cpp

# photo
 # ./tutorial_code/photo/decolorization/decolor.cpp
 # ./tutorial_code/photo/hdr_imaging/hdr_imaging.cpp
 # ./tutorial_code/photo/non_photorealistic_rendering/npr_demo.cpp
 # ./tutorial_code/photo/seamless_cloning/cloning_demo.cpp
 # ./tutorial_code/photo/seamless_cloning/cloning_gui.cpp

# ShapeDescriptors
opencv_apps_add_nodelet(find_contours src/nodelet/find_contours_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/findContours_demo.cpp
opencv_apps_add_nodelet(convex_hull src/nodelet/convex_hull_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/hull_demo.cpp
opencv_apps_add_nodelet(general_contours src/nodelet/general_contours_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/generalContours_demo2.cpp
opencv_apps_add_nodelet(contour_moments src/nodelet/contour_moments_nodelet.cpp) # ./tutorial_code/ShapeDescriptors/moments_demo.cpp
  # ./tutorial_code/ShapeDescriptors/generalContours_demo1.cpp
  # ./tutorial_code/ShapeDescriptors/pointPolygonTest_demo.cpp

# TrackingMotion
opencv_apps_add_nodelet(goodfeature_track src/nodelet/goodfeature_track_nodelet.cpp) # ./tutorial_code/TrackingMotion/goodFeaturesToTrack_Demo.cpp
  # ./tutorial_code/TrackingMotion/cornerDetector_Demo.cpp
opencv_apps_add_nodelet(corner_harris src/nodelet/corner_harris_nodelet.cpp)  # ./tutorial_code/TrackingMotion/cornerHarris_Demo.cpp
  # ./tutorial_code/TrackingMotion/cornerSubPix_Demo.cpp

# videoio
 # ./tutorial_code/video/bg_sub.cpp
if(OPENCV_HAVE_BACKGROUND_SUBTRACTION)
  opencv_apps_add_nodelet(background_subtraction src/nodelet/background_subtraction_nodelet.cpp) # ./tutorial_code/video/bg_sub.cpp, ./bgfg_segm.cpp
  if(OPENCV_HAVE_BGSEGM)
    set_source_files_properties(src/nodelet/background_subtraction_nodelet.cpp PROPERTIES
      COMPILE_DEFINITIONS OPENCV_APPS_HAVE_BGSEGM)
  endif()
endif()
 # ./tutorial_code/videoio/video-input-psnr-ssim/video-input-psnr-ssim.cpp
 # ./tutorial_code/videoio/video-write/video-write.cpp

# viz
 # ./tutorial_code/viz/creating_widgets.cpp
 # ./tutorial_code/viz/launching_viz.cpp
 # ./tutorial_code/viz/transformations.cpp
 # ./tutorial_code/viz/widget_pose.cpp

# xfeature
 # ./tutorial_code/xfeatures2D/LATCH_match.cpp

# ./3calibration.cpp
# ./autofocus.cpp
# ./bgfg_segm.cpp
# ./calibration.cpp
opencv_apps_add_nodelet(camshift src/nodelet/camshift_nodelet.cpp) # ./camshiftdemo.cpp
# ./cloning_demo.cpp
# ./cloning_gui.cpp
# ./connected_components.cpp
# ./contours2.cpp
# ./convexhull.cpp
# ./cout_mat.cpp
# ./create_mask.cpp
# ./dbt_face_detection.cpp
# ./delaunay2.cpp
# ./demhist.cpp
# ./detect_blob.cpp
opencv_apps_add_nodelet(blob_detection src/nodelet/blob_detection_nodelet.cpp) # ./detect_blob.cpp
# ./detect_mser.cpp
# ./dft.cpp
# ./distrans.cpp
# ./drawing.cpp
# ./edge.cpp
# ./em.cpp
# ./example_cmake/example.cpp
opencv_apps_add_nodelet(face_detection src/nodelet/face_detection_nodelet.cpp) # ./facedetect.cpp
opencv_apps_add_nodelet(face_recognition src/nodelet/face_recognition_nodelet.cpp)
# ./facial_features.cpp
opencv_apps_add_nodelet(fback_flow src/nodelet/fback_flow_nodelet.cpp) # ./fback.cpp
# ./ffilldemo.cpp
# ./filestorage_base64.cpp
# ./filestorage.cpp
# ./fitellipse.cpp
# ./grabcut.cpp
# ./houghcircles.cpp
# ./houghlines.cpp
# ./image_alignment.cpp
# ./image.cpp
# ./imagelist_creator.cpp
# ./image_sequence.cpp
# ./inpaint.cpp
# ./intelperc_capture.cpp
# ./kalman.cpp
# ./kmeans.cpp
# ./laplace.cpp
# ./letter_recog.cpp
opencv_apps_add_nodelet(lk_flow src/nodelet/lk_flow_nodelet.cpp) # ./lkdemo.cpp
# ./logistic_regression.cpp
# ./lsd_lines.cpp
if(OPENCV_HAVE_LSD)
  opencv_apps_add_nodelet(line_segment_detector src/nodelet/line_segment_detector_nodelet.cpp) # ./lsd_lines.cpp
endif()
# ./mask_tmpl.cpp
# ./matchmethod_orb_akaze_brisk.cpp
# ./minarea.cpp
# ./morphology2.cpp
# ./neural_network.cpp
# ./npr_demo.cpp
# ./opencv_version.cpp
# ./openni_capture.cpp
# ./pca.cpp
opencv_apps_add_nodelet(people_detect src/nodelet/people_detect_nodelet.cpp) # ./peopledetect.cpp
opencv_apps_add_nodelet(phase_corr src/nodelet/phase_corr_nodelet.cpp) # ./phase_corr.cpp
# ./points_classifier.cpp
# ./polar_transforms.cpp
opencv_apps_add_nodelet(segment_objects src/nodelet/segment_objects_nodelet.cpp) # ./segment_objects.cpp
# ./select3dobj.cpp
# ./shape_example.cpp
# ./smiledetect.cpp
# ./squares.cpp
# ./starter_imagelist.cpp
# ./starter_video.cpp
# ./stereo_calib.cpp
# ./stereo_match.cpp
# ./stitching.cpp
# ./stitching_detailed.cpp
# ./train_HOG.cpp
# ./train_svmsgd.cpp
# ./tree_engine.cpp
# ./tvl1_optical_flow.cpp
# ./videostab.cpp
opencv_apps_add_nodelet(watershed_segmentation src/nodelet/watershed_segmentation_nodelet.cpp) # ./watershed.cpp

# ros examples
opencv_apps_add_nodelet(simple_example src/nodelet/simple_example_nodelet.cpp)
opencv_apps_add_nodelet(simple_compressed_example src/nodelet/simple_compressed_example_nodelet.cpp)
# https://docs.opencv.org/4.x/d2/d0a/tutorial_introduction_to_tracker.html
# tracking requires the tracking API of OpenCV 4.5.1 or later, and KCF, CSRT and the legacy trackers are in
# opencv_contrib tracking module
if(NOT OpenCV_VERSION VERSION_LESS "4.5.1")
  set(OPENCV_HAVE_TRACKING_API TRUE)
  opencv_apps_add_nodelet(tracking src/nodelet/tracking_nodelet.cpp)
  if(TARGET opencv_tracking)
    set_source_files_properties(src/nodelet/tracking_nodelet.cpp PROPERTIES COMPILE_DEFINITIONS OPENCV_APPS_HAVE_TRACKING)
  endif()
endif()


# TApi
# ./tapi/bgfg_segm.cpp
# ./tapi/camshift.cpp
# opencv_apps_add_nodelet(equalize_histogram src/nodelet/equalize_histogram_nodelet.cpp) ./tapi/clahe.cpp
# ./tapi/dense_optical_flow.cpp
# ./tapi/hog.cpp
# ./tapi/opencl_custom_kernel.cpp
# ./tapi/pyrlk_optical_flow.cpp
# ./tapi/squares.cpp
# ./tapi/ufacedetect.cpp

# https://github.com/Itseez/opencv/blob/2.4/samples/cpp/simpleflow_demo.cpp
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  opencv_apps_add_nodelet(simple_flow src/nodelet/simple_flow_nodelet.cpp)
endif()
# https://github.com/opencv/opencv/tree/4.x/samples/dnn
if(OPENCV_HAVE_DNN)
  list(APPEND _opencv_apps_nodelet_cppfiles src/nodelet/dnn.cpp)
  opencv_apps_add_nodelet(dnn_face_detection src/nodelet/dnn_face_detection_nodelet.cpp) # ./dnn/face_detect.cpp
  opencv_apps_add_nodelet(dnn_face_recognition src/nodelet/dnn_face_recognition_nodelet.cpp) # ./dnn/face_detect.cpp
  opencv_apps_add_nodelet(dnn_object_detection src/nodelet/dnn_object_detection_nodelet.cpp) # ./dnn/object_detection.cpp
  opencv_apps_add_nodelet(dnn_tracking src/nodelet/dnn_tracking_nodelet.cpp) # ./dnn/dasiamrpn_tracker.cpp, ./dnn/nanotrack_tracker.cpp
  opencv_apps_add_nodelet(dnn_facial_expression_recognition src/nodelet/dnn_facial_expression_recognition_nodelet.cpp) # opencv_zoo facial_expression_recognition
  opencv_apps_add_nodelet(dnn_image_classification src/nodelet/dnn_image_classification_nodelet.cpp) # ./dnn/classification.cpp
  opencv_apps_add_nodelet(dnn_depth_estimation src/nodelet/dnn_depth_estimation_nodelet.cpp) # https://github.com/isl-org/MiDaS
  opencv_apps_add_nodelet(dnn_pose_estimation src/nodelet/dnn_pose_estimation_nodelet.cpp) # opencv_zoo pose_estimation_mediapipe
  opencv_apps_add_nodelet(dnn_hand_pose_estimation src/nodelet/dnn_hand_pose_estimation_nodelet.cpp) # opencv_zoo handpose_estimation_mediapipe
  opencv_apps_add_nodelet(dnn_semantic_segmentation src/nodelet/dnn_semantic_segmentation_nodelet.cpp) # ./dnn/segmentation.cpp
  opencv_apps_add_nodelet(dnn_text_detection src/nodelet/dnn_text_detection_nodelet.cpp) # ./dnn/text_detection.cpp
  opencv_apps_add_nodelet(qrcode_detection src/nodelet/qrcode_detection_nodelet.cpp) # ./cpp/qrcode.cpp
  if(OPENCV_HAVE_BARCODE)
    opencv_apps_add_nodelet(barcode_detection src/nodelet/barcode_detection_nodelet.cpp) # ./cpp/barcode.cpp
  endif()
  if(OPENCV_HAVE_ARUCO)
    opencv_apps_add_nodelet(aruco_detection src/nodelet/aruco_detection_nodelet.cpp) # ./cpp/tutorial_code/objectDetection/detect_markers.cpp
  endif()
endif()
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/bgfg_gmg.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/hybridtrackingsample.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/linemod.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/retinaDemo.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/video_dmtx.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/video_homography.cpp
# https://github.com/opencv/opencv/blob/2.4/samples/cpp/videocapture_pvapi.cpp

# detection_3d: the 3D boxes (vision_msgs) and the TF frames of the 2D detections, with a depth image; tf2_ros and
# vision_msgs are found here, not in the components of catkin above, and the node is skipped without them
find_package(tf2_ros QUIET)
find_package(vision_msgs QUIET)
if(tf2_ros_FOUND AND vision_msgs_FOUND)
  include_directories(${tf2_ros_INCLUDE_DIRS} ${vision_msgs_INCLUDE_DIRS})
  opencv_apps_add_nodelet(detection_3d src/nodelet/detection_3d_nodelet.cpp)
  set(OPENCV_APPS_HAVE_DETECTION_3D TRUE)
endif()

add_library(${PROJECT_NAME} SHARED
  src/nodelet/nodelet.cpp
  ${_opencv_apps_nodelet_cppfiles}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
if(OPENCV_APPS_HAVE_DETECTION_3D)
  target_link_libraries(${PROJECT_NAME} ${tf2_ros_LIBRARIES})
endif()
if(OPENCV_HAVE_DNN AND CURL_FOUND)
  target_include_directories(${PROJECT_NAME} PRIVATE ${CURL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES})
endif()
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${_opencv_apps_nodelet_targets}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch test scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
# rclpy on rospy for scripts/face_recognition_trainer.py, see ros1/python/rclpy/__init__.py
install(DIRECTORY ros1/python
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros1
        PATTERN "__pycache__" EXCLUDE)

## test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  if(roslaunch_VERSION VERSION_LESS "1.11.1")
    message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
  else()
    file(GLOB LAUNCH_FILES launch/*.launch)
    # the nodes not built for this OpenCV
    if(NOT OPENCV_HAVE_DNN)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_face_detection.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_face_recognition.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_object_detection.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_tracking.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_facial_expression_recognition.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_image_classification.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_depth_estimation.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_pose_estimation.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_hand_pose_estimation.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_semantic_segmentation.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/dnn_text_detection.launch
        ${CMAKE_CURRENT_SOURCE_DIR}/launch/qrcode_detection.launch)
    endif()
    if(NOT OPENCV_HAVE_BARCODE)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/barcode_detection.launch)
    endif()
    if(NOT OPENCV_APPS_HAVE_DETECTION_3D)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/detection_3d.launch)
    endif()
    if(NOT OPENCV_HAVE_ARUCO)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/aruco_detection.launch)
    endif()
    if(NOT OPENCV_HAVE_BACKGROUND_SUBTRACTION)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/background_subtraction.launch)
    endif()
    if(NOT OPENCV_HAVE_MATCH_TEMPLATE)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/match_template.launch)
    endif()
    if(NOT OPENCV_HAVE_LSD)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/line_segment_detector.launch)
    endif()
    if(NOT OPENCV_HAVE_TRACKING_API)
      list(REMOVE_ITEM LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/tracking.launch)
    endif()
    foreach(LAUNCH_FILE ${LAUNCH_FILES})
      roslaunch_add_file_check(${LAUNCH_FILE})
    endforeach()
  endif()
  add_subdirectory(test)
endif()
endif() # if($ENV{ROS_VERSION} EQUAL 2)
