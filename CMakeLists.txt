cmake_minimum_required(VERSION 3.8)
project(kinect2_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(Threads REQUIRED)

find_package(Freenect2 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(camera_info_manager REQUIRED)

add_library(kinect2_ros2_component SHARED src/kinect2_ros2_component.cpp)
target_compile_definitions(kinect2_ros2_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(kinect2_ros2_component 
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  camera_info_manager
  rclcpp_components
  ament_index_cpp
  image_transport
  OpenCV
  Freenect2)
target_link_libraries(kinect2_ros2_component
  ${Freenect2_LIBRARY})
target_include_directories(kinect2_ros2_component PRIVATE
  ${Freenect2_INCLUDE_DIRS})
rclcpp_components_register_nodes(kinect2_ros2_component "kinect2_ros2::Kinect2RosComponent")

add_executable(kinect2_ros2_node
  src/kinect2_ros2_node.cpp)
target_link_libraries(kinect2_ros2_node
  kinect2_ros2_component)
ament_target_dependencies(kinect2_ros2_node
  "rclcpp")

install(TARGETS
  kinect2_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  kinect2_ros2_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY cfg launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
