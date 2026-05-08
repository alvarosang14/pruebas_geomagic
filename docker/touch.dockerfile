FROM osrf/ros:humble-desktop AS builder

RUN apt-get update && apt-get install -y \
    udev \
    wget 

RUN apt-get update && apt-get install -y \
    freeglut3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libncurses5 \
    libncursesw5 \
    libusb-1.0-0-dev \
    libxml2 \
    && rm -rf /var/lib/apt/lists/*

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

RUN cd TouchDriver_2025_12_10 && \
    cp -r ./bin/* /usr/bin/

RUN tar -xvzf /repos/openhaptics_3.4-0-developer-edition-amd64.tar.gz
RUN cd openhaptics_3.4-0-developer-edition-amd64 && \
    chmod +x install && \
    printf "y\nq\n" | ./install && \
    ldconfig

FROM osrf/ros:humble-desktop AS final

RUN apt-get update && apt-get install -y \
    udev 

COPY --from=builder /usr/ /usr/
COPY --from=builder /etc /etc
COPY --from=builder /opt /opt

RUN ldconfig
