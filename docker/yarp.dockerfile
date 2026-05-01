FROM osrf/ros:humble-desktop AS builder

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    udev \
    ros-humble-test-msgs \
    wget 

WORKDIR /repos

# Geomagic Touch Driver
RUN  wget https://s3.us-east-1.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/Sensable/3DS/TouchDriver_2025_12_10+1.tgz
RUN wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz
   
RUN tar -xvzf /repos/TouchDriver_2025_12_10+1.tgz
RUN cd TouchDriver_2025_12_10 && \
    cp ./usr/lib/libPhantomIOLib42.so /usr/lib/libPhantomIOLib42.so && \
    cp ./usr/lib/libPhantomManagerLite.so /usr/lib/libPhantomManagerLite.so && \
    cp ./rules.d/*.rules /etc/udev/rules.d/ && \
    ldconfig

RUN tar -xvzf /repos/openhaptics_3.4-0-developer-edition-amd64.tar.gz
RUN cd openhaptics_3.4-0-developer-edition-amd64 && \
    chmod +x install && \
    printf "y\nq\n" | ./install && \
    ldconfig

# YARP
RUN git clone --branch v0.18.4      https://github.com/robotology/ycm-cmake-modules.git
RUN git clone --branch yarp-3.12    https://github.com/robotology/yarp.git
RUN git clone --branch master       https://github.com/robotology/yarp-devices-haptic.git
RUN git clone --branch master       https://github.com/robotology/yarp-devices-ros2.git

# - Build YCM
WORKDIR /repos/ycm-cmake-modules
RUN mkdir build && cd build && cmake .. && make -j$(nproc) && make install

# - Build YARP
WORKDIR /repos/yarp
RUN mkdir build && cd build && cmake -DSKIP_ACE=ON .. && make -j$(nproc) && make install

# - Build YARP devices
WORKDIR /repos/yarp-devices-haptic
RUN mkdir build && cd build && cmake -DENABLE_geomagicdriver=ON .. && make -j$(nproc) && make install

# - Build YARP devices ROS2
WORKDIR /repos/yarp-devices-ros2
RUN mkdir build && cd build && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && cmake .. && make -j$(nproc) && make install"

FROM osrf/ros:humble-desktop AS final

RUN apt-get update && apt-get install -y \
    udev \
    ros-humble-test-msgs 

COPY --from=builder /usr/ /usr/
COPY --from=builder /etc /etc
COPY --from=builder /opt /opt

RUN ldconfig

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc
