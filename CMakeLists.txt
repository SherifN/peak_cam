cmake_minimum_required(VERSION 2.8.3)
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
)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
   cfg/PeakCam.cfg
 )

catkin_package(CATKIN_DEPENDS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

# samples for ids peak library are usually installed to /usr/local/src/ids/samples/peak
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${IDS_PEAK_INCLUDE_DIR}
  ${IDS_PEAK_IPL_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
  src/peak_cam_node.cpp
  src/peak_cam.cpp 
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${IDS_PEAK_LIBRARIES}
  ${IDS_PEAK_IPL_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
