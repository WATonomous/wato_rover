ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

RUN apt-get update && apt-get install -y curl \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Copy in source code 
COPY src/robot/object_detection object_detection
COPY src/robot/odometry_spoof odometry_spoof
COPY src/robot/bringup_robot bringup_robot
COPY src/robot/camera_fallback camera_fallback
COPY src/robot/arcade_driver arcade_driver
COPY src/robot/motor_speed_controller motor_speed_controller
COPY src/wato_msgs/drivetrain_msgs drivetrain_msgs


# Scan for rosdeps
RUN apt-get update && apt-get install -y curl \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
    | grep 'apt-get install' \
    | awk '{print $3}' \
    | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# Clean up and update apt-get, then update rosdep

RUN apt-get update && apt-get install -y curl \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo rosdep update

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-get -qq update && apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list) && \
    # fallback that installs librealsense2 ROS packages if not in colcon_install_list
    apt-get -qq install -y --no-install-recommends ros-humble-librealsense2* || true

# Copy in source code from source stage
COPY src/robot/object_detection object_detection
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build


# Clean up and update apt-get, then update rosdep

RUN apt-get update && apt-get install -y curl \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN sudo apt-get clean && \
    sudo apt-get update && \
    sudo rosdep update

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
