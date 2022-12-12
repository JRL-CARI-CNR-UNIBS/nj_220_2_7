cmake_minimum_required(VERSION 3.0.2)
project(nj_220_2_7_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nj_220_2_7_gazebo
#  CATKIN_DEPENDS gazebo_ros roscpp
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
