ARG BASE_IMAGE=ghcr.io/watonomous/robot_base/base:humble-ubuntu22.04

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
# COPY src/robot/yolo_inference ./yolo_inference
# COPY src/robot/object_detection object_detection
COPY src/robot/odometry_spoof odometry_spoof
COPY src/robot/gps_sim gps_sim
COPY src/robot/imu_sim imu_sim
COPY src/robot/localization localization
COPY src/robot/costmap costmap
COPY src/robot/map_memory map_memory
COPY src/robot/planner planner
COPY src/robot/control control
COPY src/robot/bringup_robot bringup_robot

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
    | (grep 'apt-get install' || true) \
    | awk '{print $3}' \
    | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# ADD MORE DEPENDENCIES HERE

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends "$(cat /tmp/colcon_install_list)"

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Install Python deps and preprocess elevation data into a cost grid CSV
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && \
    rm -rf /var/lib/apt/lists/*
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY assets/fargate_utah_mdrs.las /tmp/fargate_utah_mdrs.las
COPY scripts/preprocess_elevation.py /tmp/preprocess_elevation.py
RUN python3 /tmp/preprocess_elevation.py \
    --input /tmp/fargate_utah_mdrs.las \
    --output /elevation_grid.csv \
    --width 60 --height 60 --resolution 0.5 \
    --origin_x -15.0 --origin_y -15.0 \
    --max_slope 2.0 \
    && rm -f /tmp/fargate_utah_mdrs.las /tmp/preprocess_elevation.py

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/"$ROS_DISTRO"/setup.sh && \
    colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base "$WATONOMOUS_INSTALL"

# Source and Build Artifact Cleanup
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]
