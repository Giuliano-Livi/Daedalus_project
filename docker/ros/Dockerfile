FROM ros:humble-ros-base
ARG DEBIAN_FRONTEND=noninteractive
# px4_msgs deve corrispondere al PX4 dell'immagine px4io/px4-sitl-gazebo:latest (main 61907962,
# 11/09/2026): con px4_msgs piu' recenti cambiano le versioni dei messaggi (es. VehicleStatus v5)
# e i topic /fmu/out/*_v4 non arrivano piu'. Aggiornare insieme all'immagine PX4.
ARG PX4_MSGS_REF=57f88be
ARG XRCE_AGENT_TAG=v2.4.2
ARG WITH_GZ=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential cmake python3-pip python3-colcon-common-extensions \
      curl ca-certificates gnupg \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic + bridge ROS (solo per simulazione, repo OSRF)
RUN if [ "$WITH_GZ" = "1" ]; then \
      . /etc/os-release && \
      curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
      apt-get update && \
      apt-get install -y --no-install-recommends gz-harmonic ros-humble-ros-gzharmonic && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# RTAB-Map (graph SLAM RGB-D) + static TF per i frame della camera
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rtabmap-ros ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -U empy==3.3.4 pyros-genmsg setuptools==65.7.0

WORKDIR /deps_ws/src
RUN git clone --depth 1 --branch ${XRCE_AGENT_TAG} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
 && git clone https://github.com/PX4/px4_msgs.git && git -C px4_msgs checkout ${PX4_MSGS_REF}
WORKDIR /deps_ws
RUN . /opt/ros/humble/setup.sh && colcon build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
