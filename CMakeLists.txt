cmake_minimum_required(VERSION 3.0.2)
project(handsfree_ros_imu_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES handsfree_IMU_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(hfi_b6 src/hfi_b6_ros.cpp)
target_link_libraries(hfi_b6 ${catkin_LIBRARIES})

add_executable(hfi_b9 src/hfi_b9_ros.cpp)
target_link_libraries(hfi_b9 ${catkin_LIBRARIES})