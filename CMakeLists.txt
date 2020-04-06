cmake_minimum_required(VERSION 2.8.3)
project(mir_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
    mir_actions
    mir_description
    mir_driver
    mir_dwb_critics
    mir_gazebo
    mir_msgs
    mir_navigation
    octomap_server
    laserscan_to_pointcloud
    roboskel_ros_resources
    roscpp
    visualization_msgs
    sensor_msgs
    pcl_ros
    pcl_conversions
    nav_msgs
    std_msgs
    std_srvs
    octomap_ros
    octomap_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
    DEPENDS OCTOMAP
)

set(LINK_LIBS
    ${OCTOMAP_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(octomap_map_changes src/TrackingOctomapServer.cpp)
target_link_libraries(octomap_map_changes ${LINK_LIBS})

install(TARGETS
    octomap_map_changes
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY
    launch
    maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

