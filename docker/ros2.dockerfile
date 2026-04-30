FROM osrf/ros:humble-desktop AS builder

# clone the repository
RUN apt-get update && apt-get install -y git

# Ros2 workspace
RUN mkdir /ros2_ws
WORKDIR /ros2_ws

# Install dependencies
RUN git clone --branch master https://github.com/roboticslab-uc3m/abb_egm_driver.git /ros2_ws/src
RUN git clone --branch main https://github.com/alvarosang14/pruebas_geomagic.git /tmp/pruebas_geomagic
RUN cp -r /tmp/pruebas_geomagic/ros2_interfaces /ros2_ws/src/
RUN rm -rf /tmp/pruebas_geomagic

RUN cd /ros2_ws && colcon build 

FROM osrf/ros:humble-desktop AS final

# Build the workspace
COPY --from=builder /ros2_ws /ros2_ws
WORKDIR /ros2_ws

# Entrypoint
ENTRYPOINT ["source", "/ros2_ws/install/setup.bash"]