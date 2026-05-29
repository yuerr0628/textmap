cmake_minimum_required(VERSION 3.0.2)
project(textmap_reproduction)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_filters
  parking_slot_detection
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(g2o REQUIRED)
find_package(CURL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport message_filters parking_slot_detection
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

add_library(textmap_core
  src/json_io.cpp
  src/geometry.cpp
  src/single_vehicle_mapper.cpp
  src/qwen_api.cpp
  src/llm_map_optimizer.cpp
  src/crowd_fusion.cpp
  src/localization.cpp
)

target_link_libraries(textmap_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${CURL_LIBRARIES}
  g2o_core
  g2o_solver_eigen
  g2o_stuff
  g2o_types_slam2d
)

add_executable(build_single_vehicle_map tools/build_single_vehicle_map.cpp)
target_link_libraries(build_single_vehicle_map textmap_core)

add_executable(optimize_and_fuse_maps tools/optimize_and_fuse_maps.cpp)
target_link_libraries(optimize_and_fuse_maps textmap_core)

add_executable(localize_frame tools/localize_frame.cpp)
target_link_libraries(localize_frame textmap_core)

add_executable(single_vehicle_mapping_node
  src/ros_pose_provider.cpp
  src/ros_single_vehicle_mapping_node.cpp
)
target_link_libraries(single_vehicle_mapping_node textmap_core)
add_dependencies(single_vehicle_mapping_node ${catkin_EXPORTED_TARGETS} parking_slot_detection_generate_messages_cpp)
