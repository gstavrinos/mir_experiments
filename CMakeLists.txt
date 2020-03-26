cmake_minimum_required(VERSION 2.8.3)
project(mir_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mir_actions
  mir_description
  mir_driver
  mir_dwb_critics
  mir_gazebo
  mir_msgs
  mir_navigation
  roboskel_ros_resources
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
