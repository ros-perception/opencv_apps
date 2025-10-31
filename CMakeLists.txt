cmake_minimum_required(VERSION 3.8)
project(opencv_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++14 for ROS2
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV VERSION: ${OpenCV_VERSION}")

# Generate messages and services
set(msg_files
  "msg/Point2D.msg"
  "msg/Point2DStamped.msg"
  "msg/Point2DArray.msg"
  "msg/Point2DArrayStamped.msg"
  "msg/Rect.msg"
  "msg/RectArray.msg"
  "msg/RectArrayStamped.msg"
  "msg/Flow.msg"
  "msg/FlowStamped.msg"
  "msg/FlowArray.msg"
  "msg/FlowArrayStamped.msg"
  "msg/Size.msg"
  "msg/Face.msg"
  "msg/FaceArray.msg"
  "msg/FaceArrayStamped.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  "msg/LineArrayStamped.msg"
  "msg/RotatedRect.msg"
  "msg/RotatedRectStamped.msg"
  "msg/RotatedRectArray.msg"
  "msg/RotatedRectArrayStamped.msg"
  "msg/Circle.msg"
  "msg/CircleArray.msg"
  "msg/CircleArrayStamped.msg"
  "msg/Moment.msg"
  "msg/MomentArray.msg"
  "msg/MomentArrayStamped.msg"
  "msg/Contour.msg"
  "msg/ContourArray.msg"
  "msg/ContourArrayStamped.msg"
)

set(srv_files
  "srv/FaceRecognitionTrain.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create a list of dependencies
set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  image_transport
)

# Macro to add nodes
macro(opencv_apps_add_node node_name nodelet_cppfile)
  add_executable(${node_name}_exe ${nodelet_cppfile} src/nodelet/nodelet.cpp)
  ament_target_dependencies(${node_name}_exe ${dependencies})
  target_link_libraries(${node_name}_exe ${OpenCV_LIBRARIES})
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${node_name}_exe "${cpp_typesupport_target}")
  set_target_properties(${node_name}_exe PROPERTIES OUTPUT_NAME ${node_name})

  install(TARGETS ${node_name}_exe
    DESTINATION lib/${PROJECT_NAME})
endmacro()

# Add nodes
opencv_apps_add_node(edge_detection src/nodelet/edge_detection_nodelet.cpp)
opencv_apps_add_node(camshift src/nodelet/camshift_nodelet.cpp)
# opencv_apps_add_node(face_detection src/nodelet/face_detection_nodelet.cpp)
# opencv_apps_add_node(hough_lines src/nodelet/hough_lines_nodelet.cpp)
# opencv_apps_add_node(hough_circles src/nodelet/hough_circles_nodelet.cpp)
# opencv_apps_add_node(find_contours src/nodelet/find_contours_nodelet.cpp)
# opencv_apps_add_node(convex_hull src/nodelet/convex_hull_nodelet.cpp)
# opencv_apps_add_node(goodfeature_track src/nodelet/goodfeature_track_nodelet.cpp)
# opencv_apps_add_node(corner_harris src/nodelet/corner_harris_nodelet.cpp)
# opencv_apps_add_node(fback_flow src/nodelet/fback_flow_nodelet.cpp)
# opencv_apps_add_node(lk_flow src/nodelet/lk_flow_nodelet.cpp)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
