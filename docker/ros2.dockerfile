FROM osrf/ros:humble-desktop AS builder

# clone the repository
RUN apt-get update && apt-get install -y \
    git \
    pip

RUN pip install ABBRobotEGM

# Ros2 workspace
RUN mkdir /ros2_ws
WORKDIR /ros2_ws

# Install dependencies
RUN git clone --branch master https://github.com/roboticslab-uc3m/abb_egm_driver.git /ros2_ws/src/abb_egm_driver
RUN git clone --branch master https://github.com/roboticslab-uc3m/jr3_driver.git     /ros2_ws/src/jr3_driver
RUN git clone --branch main https://github.com/alvarosang14/pruebas_geomagic.git /tmp/pruebas_geomagic
RUN cp -r /tmp/pruebas_geomagic/ros2_interfaces /ros2_ws/src/
RUN rm -rf /tmp/pruebas_geomagic

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  cd /ros2_ws && \
                  colcon build --symlink-install"

FROM osrf/ros:humble-desktop AS final

COPY --from=builder /ros2_ws /ros2_ws
COPY --from=builder /usr/local/lib/python3.10/dist-packages /usr/local/lib/python3.10/dist-packages
COPY --from=builder /usr/local/bin /usr/local/bin

WORKDIR /ros2_ws

RUN sed -i '/^exec "\$@"/i source /ros2_ws/install/setup.bash' /ros_entrypoint.sh