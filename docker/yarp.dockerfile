FROM touch:latest AS builder

# Compiladores + headers para linkar contra OpenHaptics y deps de ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    libusb-1.0-0-dev \
    ros-humble-test-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /repos

RUN git clone --branch v0.18.4          https://github.com/robotology/ycm-cmake-modules.git \
    && git clone --branch yarp-3.12     https://github.com/robotology/yarp.git \
    && git clone --branch master        https://github.com/robotology/yarp-devices-haptic.git \
    && git clone --branch master        https://github.com/robotology/yarp-devices-ros2.git

# YCM
RUN cd /repos/ycm-cmake-modules \
    && mkdir build && cd build \
    && cmake .. \
    && cmake --build . -j$(nproc) \
    && cmake --install .

# YARP core
RUN cd /repos/yarp \
    && mkdir build && cd build \
    && cmake .. -DSKIP_ACE=ON \
    && cmake --build . -j$(nproc) \
    && cmake --install . \
    && ldconfig

# yarp-devices-haptic con driver Geomagic
RUN cd /repos/yarp-devices-haptic \
    && mkdir build && cd build \
    && cmake .. -DENABLE_GEOMAGIC=ON \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# yarp-devices-ros2 (necesita ROS sourced)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
        cd /repos/yarp-devices-ros2 && \
        mkdir build && cd build && \
        cmake .. && \
        make -j$(nproc) && \
        make install" \
    && ldconfig

FROM touch:latest AS final

# Únicas deps runtime que NO vienen ya en la base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-test-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/ /usr/local/

RUN ldconfig \
    && echo "source /opt/ros/humble/setup.bash"  >> /etc/bash.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

WORKDIR /ros2_ws
