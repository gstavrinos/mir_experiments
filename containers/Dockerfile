# Docker container to easily test the mir_experiments functionality
# At some point I will have to use docker compose to run multiple ros nodes
# Build with: sudo docker build -t mir_experiments:0.2 <mir_experiments-location>/containers/Dockerfile
# Run with: sudo docker run --net=host mir_experiments:0.2
FROM ros:melodic-ros-core-bionic

LABEL maintainer="George Stavrinos <gstavrinos@iit.demokritos.gr>"

RUN apt-get update && apt-get install -y \
    python-catkin-tools \
    ros-melodic-costmap-2d \
    ros-melodic-costmap-queue \
    ros-melodic-desktop-full \
    ros-melodic-octomap-mapping \
    ros-melodic-octomap-msgs \
    ros-melodic-octomap-ros \
    ros-melodic-octomap-server \
    ros-melodic-rospy-message-converter \
    ros-melodic-gazebo-ros-control && \
    rm -rf /var/lib/apt-get/lists/*

RUN mkdir -p /opt/catkin_ws/src

RUN apt-get install -y \
        build-essential \
        git \
        python-rosdep && \
    rosdep init && \
    rosdep update && \
    cd /opt/catkin_ws/src && \
    git clone https://github.com/roboskel/roboskel_msgs && \
    git clone https://github.com/roboskel/roboskel_ros_resources && \
    git clone https://github.com/gstavrinos/mir_robot && \
    git clone https://github.com/gstavrinos/mir_experiments && \
    git clone https://github.com/gstavrinos/ros_rvv && \
    git clone https://github.com/carlosmccosta/laserscan_to_pointcloud && \
    cd .. && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make --pkg roboskel_msgs" && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; source /opt/catkin_ws/devel/setup.bash; catkin_make" && \
    apt-get purge -y \
        build-essential \
        git && \
    rm -rf /var/lib/apt-get/lists/*

RUN sed --in-place --expression \
    '$isource "/opt/catkin_ws/devel/setup.bash"' \
    /ros_entrypoint.sh

# EXPOSE 11311/udp
# EXPOSE 11311/tcp

# ENTRYPOINT roslaunch mir_experiments changes_based_on_tidy_small_room_depth_camera.launch
# CMD ["roslaunch", "mir_experiments", "changes_based_on_tidy_small_room_depth_camera.launch"]

