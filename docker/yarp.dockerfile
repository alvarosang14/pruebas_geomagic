FROM osrf/ros:humble-desktop AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget 

WORKDIR /repos

# Geomagic Touch Driver
RUN  wget https://s3.us-east-1.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/Sensable/3DS \
/TouchDriver_2025_12_10+1.tgz    
RUN tar -xvzf /repos/TouchDriver_2025_12_10+1.tgz && cd TouchDriver_2025_12_10+1 && ./install.sh

# YARP
RUN git clone --branch v0.18.4 https://github.com/robotology/ycm-cmake-modules.git
RUN git clone --branch v3.12 https://github.com/robotology/yarp.git
RUN git clone --branch master https://github.com/robotology/yarp-devices-haptic.git
RUN git clone --branch master https://github.com/robotology/yarp-devices-ros2.git

# - Build YARP
WORKDIR /repos/yarp
RUN mkdir build && cd build && cmake .. && make -j$(nproc) && make install

# - Build YARP devices
WORKDIR /repos/yarp-devices-haptic
RUN mkdir build && cd build && cmake -DENABLE_geomagicdriver=ON .. && make -j$(nproc) && make install

# - Build YARP devices ROS2
WORKDIR /repos/yarp-devices-ros2
RUN cd ros2_interfaces && colcon build --symlink-install
RUN mkdir build && cd build && cmake .. && make -j$(nproc) && make install

FROM osrf/ros:humble-desktop AS final

COPY --from=builder /usr/ /usr/

COPY --from=builder /repos/yarp-devices-ros2/ros2_interfaces/install /ros2_ws/install

RUN ldconfig

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc
