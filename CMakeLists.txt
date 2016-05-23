cmake_minimum_required(VERSION 2.8.3)
project(opencv_apps)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure message_generation image_transport nodelet roscpp)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Components: ${OpenCV_LIB_COMPONENTS}")
if(OpenCV_VERSION VERSION_LESS "3.0" OR TARGET opencv_optflow)
  set(OPENCV_HAVE_OPTFLOW TRUE)
endif()

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/EdgeDetection.cfg cfg/HoughLines.cfg cfg/HoughCircles.cfg
  cfg/FindContours.cfg cfg/ConvexHull.cfg cfg/GeneralContours.cfg cfg/ContourMoments.cfg
  cfg/FaceDetection.cfg
  cfg/GoodfeatureTrack.cfg
  #
  cfg/CamShift.cfg
  cfg/FBackFlow.cfg
  cfg/LKFlow.cfg
  cfg/PeopleDetect.cfg
  cfg/PhaseCorr.cfg
  cfg/SegmentObjects.cfg
  cfg/SimpleFlow.cfg
  cfg/WatershedSegmentation.cfg
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
   RectArrayStamped.msg
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
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs
#                DEPENDS OpenCV
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a cpp library
if(OPENCV_HAVE_OPTFLOW)
  list(APPEND ${PROJECT_NAME}_EXTRA_FILES src/nodelet/simple_flow_nodelet.cpp)
endif()

## Macro to add nodelets
macro(opencv_apps_add_nodelet node_name nodelet_name nodelet_cppfile)
  set(NODE_NAME ${node_name})
  set(NODELET_NAME ${nodelet_name})
  configure_file(src/node/standalone_nodelet_exec.cpp.in ${node_name}.cpp @ONLY)
  add_executable(${node_name}_exe ${node_name}.cpp)
  SET_TARGET_PROPERTIES(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})
  target_link_libraries(${node_name}_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  list(APPEND _opencv_apps_nodelet_cppfiles ${nodelet_cppfile})
  list(APPEND _opencv_apps_nodelet_targets ${node_name}_exe)
endmacro()

# https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code
# ImgTrans
opencv_apps_add_nodelet(edge_detection edge_detection/edge_detection src/nodelet/edge_detection_nodelet.cpp)
opencv_apps_add_nodelet(hough_lines hough_lines/hough_lines src/nodelet/hough_lines_nodelet.cpp)
opencv_apps_add_nodelet(hough_circles hough_circles/hough_circles src/nodelet/hough_circles_nodelet.cpp)
# ShapeDescriptors
opencv_apps_add_nodelet(find_contours find_contours/find_contours src/nodelet/find_contours_nodelet.cpp)
opencv_apps_add_nodelet(convex_hull convex_hull/convex_hull src/nodelet/convex_hull_nodelet.cpp)
opencv_apps_add_nodelet(general_contours general_contours/general_contours src/nodelet/general_contours_nodelet.cpp)
opencv_apps_add_nodelet(contour_moments contour_moments/contour_moments src/nodelet/contour_moments_nodelet.cpp)
# objectDetection
opencv_apps_add_nodelet(face_detection face_detection/face_detection src/nodelet/face_detection_nodelet.cpp)
opencv_apps_add_nodelet(people_detect people_detect/people_detect src/nodelet/people_detect_nodelet.cpp)
  # bgfg_gmg.cpp
  # bgfg_segm.cpp
  # calibration.cpp
# TrackingMotion
opencv_apps_add_nodelet(goodfeature_track goodfeature_track/goodfeature_track src/nodelet/goodfeature_track_nodelet.cpp)
# Samples
opencv_apps_add_nodelet(camshift camshift/camshift src/nodelet/camshift_nodelet.cpp)
opencv_apps_add_nodelet(simple_example simple_example/simple_example src/nodelet/simple_example_nodelet.cpp)
opencv_apps_add_nodelet(simple_compressed_example simple_compressed_example/simple_compressed_example src/nodelet/simple_compressed_example_nodelet.cpp)
# Optical Flow
opencv_apps_add_nodelet(fback_flow fback_flow/fback_flow src/nodelet/fback_flow_nodelet.cpp)
  # fback.cpp
  # hybridtrackingsample.cpp
  # image_sequence.cpp
  # intelperc_capture.cpp
  # laplace.cpp
  # linemod.cpp
  # lkdemo.cpp
opencv_apps_add_nodelet(lk_flow lk_flow/lk_flow src/nodelet/lk_flow_nodelet.cpp)
# Others
opencv_apps_add_nodelet(phase_corr phase_corr/phase_corr src/nodelet/phase_corr_nodelet.cpp)
  # phase_corr.cpp
  # retinaDemo.cpp
  # segment_objects.cpp
# Segmentation
opencv_apps_add_nodelet(segment_objects segment_objects/segment_objects src/nodelet/segment_objects_nodelet.cpp)
  # select3dobj.cpp
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  opencv_apps_add_nodelet(simple_flow simple_flow/simple_flow src/nodelet/simple_flow_nodelet.cpp)
endif()
  # starter_video.cpp
  # videocapture_pvapi.cpp
  # video_dmtx.cpp
  # video_homography.cpp
opencv_apps_add_nodelet(watershed_segmentation watershed_segmentation/watershed_segmentation src/nodelet/watershed_segmentation_nodelet.cpp)

add_library(${PROJECT_NAME} SHARED
  src/nodelet/nodelet.cpp
  ${_opencv_apps_nodelet_cppfiles}
  # https://github.com/Itseez/opencv/blob/2.4/samples/cpp
  # 3calibration.cpp
  # bagofwords_classification.cpp
  # bgfg_gmg.cpp
  # bgfg_segm.cpp
  # brief_match_test.cpp
  # build3dmodel.cpp
  # calibration_artificial.cpp
  # calibration.cpp
  # chamfer.cpp
  # connected_components.cpp
  # contours2.cpp
  # convexhull.cpp
  # cout_mat.cpp
  # delaunay2.cpp
  # demhist.cpp
  # descriptor_extractor_matcher.cpp
  # detection_based_tracker_sample.cpp
  # detector_descriptor_evaluation.cpp
  # detector_descriptor_matcher_evaluation.cpp
  # dft.cpp
  # distrans.cpp
  # drawing.cpp
  # edge.cpp
  # em.cpp
  # fabmap_sample.cpp
  # facerec_demo.cpp
  # facial_features.cpp
  # ffilldemo.cpp
  # filestorage.cpp
  # fitellipse.cpp
  # freak_demo.cpp
  # gencolors.cpp
  # generic_descriptor_match.cpp
  # grabcut.cpp
  # houghcircles.cpp
  # houghlines.cpp
  # hybridtrackingsample.cpp
  # image.cpp
  # imagelist_creator.cpp
  # image_sequence.cpp
  # inpaint.cpp
  # intelperc_capture.cpp
  # kalman.cpp
  # kmeans.cpp
  # laplace.cpp
  # latentsvm_multidetect.cpp
  # letter_recog.cpp
  # linemod.cpp
  # logpolar_bsm.cpp
  # matcher_simple.cpp
  # matching_to_many_images.cpp
  # meanshift_segmentation.cpp
  # minarea.cpp
  # morphology2.cpp
  # opencv_version.cpp
  # OpenEXRimages_HDR_Retina_toneMapping.cpp
  # OpenEXRimages_HDR_Retina_toneMapping_video.cpp
  # openni_capture.cpp
  # pca.cpp
  # points_classifier.cpp
  # retinaDemo.cpp
  # rgbdodometry.cpp
  ## select3dobj.cpp
###
### simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
###
###  src/nodelet/simple_flow_nodelet.cpp
  # squares.cpp
  # starter_imagelist.cpp
  # starter_video.cpp
  # stereo_calib.cpp
  # stereo_match.cpp
  # stitching.cpp
  # stitching_detailed.cpp
  # tvl1_optical_flow.cpp
  ## videocapture_pvapi.cpp
  ## video_dmtx.cpp
  # video_homography.cpp
  # videostab.cpp
  ${${PROJECT_NAME}_EXTRA_FILES}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
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

## test
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)
  add_subdirectory(test)
endif()
