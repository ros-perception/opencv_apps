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

# Training data for objectness
catkin_download_test_data(ObjNessB2W8HSV.idx.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8HSV.idx.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 e066c100d60246a3911d4559182d9d2a
  )
catkin_download_test_data(ObjNessB2W8HSV.wS1.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8HSV.wS1.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 728507d99521d7dba9b0eb114ccbb830
  )
catkin_download_test_data(ObjNessB2W8HSV.wS2.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8HSV.wS2.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 790e27251267d86168a12f2bd2d96f8d
  )
catkin_download_test_data(ObjNessB2W8I.idx.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8I.idx.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 9425dd4d31521fced82aeb6fc56ce4d5
  )
catkin_download_test_data(ObjNessB2W8I.wS1.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8I.wS1.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 a04d4b4504887fc16800b8b42bac9e70
  )
catkin_download_test_data(ObjNessB2W8I.wS2.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8I.wS2.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 f2e2f5726e352bfa16224066e2bdc7ad
  )
catkin_download_test_data(ObjNessB2W8MAXBGR.idx.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8MAXBGR.idx.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 ef2fbd5da0ffb5fe4332685b8529dc5c
  )
catkin_download_test_data(ObjNessB2W8MAXBGR.wS1.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8MAXBGR.wS1.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 cbe8147fca9a5885b7bb25d38fa5f4d1
  )
catkin_download_test_data(ObjNessB2W8MAXBGR.wS2.yml.gz https://github.com/opencv/opencv_contrib/raw/3.2.0/modules/saliency/samples/ObjectnessTrainedModel/ObjNessB2W8MAXBGR.wS2.yml.gz
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 02b76364df35cef862da041585b537de
  )

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
add_rostest(test-equalize_histogram.test ARGS gui:=false)
add_rostest(test-objectness.test ARGS gui:=false)

add_rostest(test-hough_lines.test ARGS gui:=false)
add_rostest(test-hough_circles.test ARGS gui:=false)
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
else()
  add_rostest(test-face_detection.test ARGS gui:=false use_opencv3:=true use_opencv3_3:=true)
  add_rostest(test-face_recognition.test ARGS gui:=false use_opencv3:=true use_opencv3_3:=true)
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
add_rostest(test-watershed_segmentation.test ARGS gui:=false)

add_rostest(test-simple_example.test ARGS gui:=false)
add_rostest(test-simple_compressed_example.test ARGS gui:=false)

if(roslaunch_VERSION VERSION_LESS "1.11.1")
  message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
else()
  file(GLOB LAUNCH_FILES *.test)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()
