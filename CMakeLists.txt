cmake_minimum_required(VERSION 3.5.1)
project(peak_cam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(ids_peak REQUIRED)
find_package(ids_peak_ipl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nodelet
)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
   cfg/PeakCam.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nodelet
)

# samples for ids peak library are usually installed to /usr/local/src/ids/samples/peak
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${IDS_PEAK_INCLUDE_DIR}
  ${IDS_PEAK_IPL_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/peak_cam_node.cpp
)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${IDS_PEAK_LIBRARIES}
  ${IDS_PEAK_IPL_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
