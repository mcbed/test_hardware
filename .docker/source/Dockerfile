ARG ROS_DISTRO="humble"
FROM ros:$ROS_DISTRO
ARG BRANCH="main"

ENV ROS_UNDERLAY /root/ws_ros2/install
WORKDIR $ROS_UNDERLAY/../src

ADD https://raw.githubusercontent.com/mcbed/test_hardware/main/test_hardware.repos test_hardware.repos
ADD . $ROS_UNDERLAY/../src
RUN vcs import < test_hardware.repos

RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

RUN cd $ROS_UNDERLAY/.. && \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        colcon build

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "$ROS_UNDERLAY/setup.bash"' \
      /ros_entrypoint.sh
