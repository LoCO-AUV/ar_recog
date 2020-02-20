cmake_minimum_required(VERSION 2.8.3)
project(ar_recog)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -Wall -O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  cv_bridge
  visualization_msgs
#  std_srvs
#  geometry_msgs
#  sensor_msgs
  image_transport
  tf
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)

include_directories(${PROJECT_SOURCE_DIR}/src/ARToolKit/include ${PROJECT_SOURCE_DIR}/src/ARToolKit/examples/loadMultiple)
link_directories(${PROJECT_SOURCE_DIR}/src/ARToolKit/lib)

add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/src/ARToolKit/examples/loadMultiple/object.c
        COMMAND ${PROJECT_SOURCE_DIR}/getAndBuildARToolkit.py
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
    Tag.msg
    Tags.msg
)

add_service_files(
  FILES
    CalibrateDistance.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
#    std_srvs
#    geometry_msgs
#    sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
  visualization_msgs
#    std_srvs
#    geometry_msgs
#    sensor_msgs
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  lib
  ${OpenCV_LIBRARY_DIRS}
)

add_executable(ar_compute_pose src/ar_compute_pose.cpp)
target_link_libraries(ar_compute_pose
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
add_dependencies(ar_compute_pose OpenCV)
add_dependencies(ar_compute_pose ${PROJECT_NAME}_generate_messages_cpp)


#rosbuild_add_executable(ar_recog src/ar_recog.cpp src/ARToolKit/examples/loadMultiple/object.c)
add_executable(ar_recog src/ar_recog.cpp src/load_model.c)
target_link_libraries(ar_recog AR ${catkin_LIBRARIES})
add_dependencies(ar_recog artoolkit ${PROJECT_NAME}_generate_messages_cpp)
add_custom_target(artoolkit DEPENDS ${PROJECT_SOURCE_DIR}/src/ARToolKit/examples/loadMultiple/object.c)
