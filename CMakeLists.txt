cmake_minimum_required(VERSION 2.8.3)
project(redwall_arm_vision)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  geometry_msgs
  sensor_msgs
  OpenCV
  image_transport
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES redwall_arm_vision
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(1.ros_cv_msg src/1.ros_cv_msg.cpp)
target_link_libraries(1.ros_cv_msg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(7.open_cv_aoi src/7.open_cv_aoi.cpp)
target_link_libraries(7.open_cv_aoi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(44.平均背景法 src/44.平均背景法.cpp)
target_link_libraries(44.平均背景法 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(45.codebook求背景 src/45.codebook求背景.cpp)
target_link_libraries(45.codebook求背景 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(46.连通分量进行前景处理 src/46.连通分量进行前景处理.cpp)
target_link_libraries(46.连通分量进行前景处理 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(47.BackgroundSubtractor src/47.BackgroundSubtractor.cpp)
target_link_libraries(47.BackgroundSubtractor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(adjust_threshold src/adjust_threshold.cpp)
target_link_libraries(adjust_threshold ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
