# Docker container to easily test the mir_experiments functionality
# At some point I will have to use docker compose to run multiple ros nodes
# Build with: sudo docker build -t mir_experiments:0.1 <mir_experiments-location>/containers/Dockerfile
# Run with: sudo docker run --net=host mir_experiments:0.1
FROM ros:melodic-ros-core-bionic
MAINTAINER George Stavrinos gstavrinos@iit.demokritos.gr

RUN apt-get update && \
apt-get install ros-melodic-desktop-full -y && \
apt-get install ros-melodic-costmap-2d -y && \
apt-get install ros-melodic-costmap-queue -y && \
apt-get install ros-melodic-octomap-server -y && \
apt-get install ros-melodic-octomap-ros -y && \
apt-get install ros-melodic-octomap-mapping -y && \
apt-get install ros-melodic-octomap-msgs -y && \
apt-get install ros-melodic-rospy-message-converter -y && \
apt-get install python-catkin-tools -y && \
apt-get install git -y && \
apt-get install build-essential -y && \
apt-get install python-rosdep

RUN rosdep init && \
rosdep update && \
mkdir -p /opt/catkin_ws/src && \
cd /opt/catkin_ws/src && \
git clone https://github.com/roboskel/roboskel_msgs && \
git clone https://github.com/roboskel/roboskel_ros_resources && \
git clone https://github.com/gstavrinos/mir_robot && \
git clone https://github.com/gstavrinos/mir_experiments && \
git clone https://github.com/gstavrinos/ros_rvv && \
git clone https://github.com/carlosmccosta/laserscan_to_pointcloud && \
cd .. && \
rosdep install --from-paths src --ignore-src -r -y && \
rm -rf /var/lib/apt-get/lists/* && \
/bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make --pkg roboskel_msgs" && \
/bin/bash -c "source /opt/ros/melodic/setup.bash; source /opt/catkin_ws/devel/setup.bash; catkin_make"

RUN sed --in-place --expression \
      '$isource "/opt/catkin_ws/devel/setup.bash"' \
      /ros_entrypoint.sh

# EXPOSE 11311/udp
# EXPOSE 11311/tcp

# ENTRYPOINT roslaunch mir_experiments changes_based_on_tidy_small_room_depth_camera.launch
CMD ["roslaunch", "mir_experiments", "changes_based_on_tidy_small_room_depth_camera.launch"]

