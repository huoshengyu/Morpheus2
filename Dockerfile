FROM nvidia/cuda:12.9.0-base-ubuntu20.04 AS base
# RUN  rm /etc/apt/sources.list.d/nvidia-ml.list && apt-get clean && apt-get update

SHELL ["/bin/bash", "-c"]

# Install git and wget
RUN apt-get update && \
    apt-get install git wget udev -y

# Install Miniconda
RUN wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3.sh
RUN bash ~/miniconda3.sh -b -p ~/miniconda3
RUN rm ~/miniconda3.sh

RUN export PATH=~/miniconda3/bin:$PATH

# Minimal setup
ENV ROS_DISTRO=noetic
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y \
    locales \
    lsb-release \
    curl \
    && rm -rf /var/lib/apt/lists/*
RUN dpkg-reconfigure locales

# Install ROS Noetic Desktop Full
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstools \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-rosserial \
#     ros-noetic-rosserial-python \
#     ros-noetic-rosserial-arduino \
#     && rm -rf /var/lib/apt/lists/*
# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update --rosdistro $ROS_DISTRO

# Source ROS setup files on container startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

FROM base AS dev

# Install ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-noetic-moveit \
    ros-noetic-cartesian-control-msgs \
    ros-noetic-teleop-twist-keyboard \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory in the container
WORKDIR /root/catkin_ws/

# Copy the morpheus repo
COPY ./ ./src/

# Install general dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    python3-catkin-tools \
    python3-zipp \
    python-is-python3 \
    libspnav-dev \
    spacenavd \
    ros-noetic-spacenav-node \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies 
# (Relatively error-prone dependencies installed individually for readability of error messages)
RUN pip install --upgrade pip
RUN pip install --upgrade \ 
    pyserial \
    pymodbus===2.1.0 \
    numpy \
    numpy-quaternion \
    scipy \
    readchar \
    pynput \
    pygame-ce
RUN pip install --upgrade importlib_metadata
RUN pip install --upgrade six
RUN pip install --upgrade setuptools
RUN pip install --upgrade PyQt6
RUN pip install --upgrade modern_robotics

# Install GELLO dependencies
RUN pip install -r ./src/gello_software/requirements.txt
RUN pip install -e ./src/gello_software/. --use-pep517
RUN pip install -e ./src/gello_software/third_party/DynamixelSDK/python/.
RUN pip install pylsl

# --- build liblsl from source (for pylsl) ---
RUN apt-get update && apt-get install -y \
      git cmake g++ libpugixml-dev && \
    rm -rf /var/lib/apt/lists/*

# cmake in Ubuntu 20.04 is too old for liblsl, so grab a newer one
RUN cd /tmp && \
    wget https://github.com/Kitware/CMake/releases/download/v3.29.6/cmake-3.29.6-linux-x86_64.tar.gz && \
    tar -xzf cmake-3.29.6-linux-x86_64.tar.gz && \
    mv cmake-3.29.6-linux-x86_64 /opt/cmake-3.29

# clone and build liblsl
RUN cd /root && \
    git clone https://github.com/sccn/liblsl.git && \
    cd liblsl && \
    mkdir build && cd build && \
    /opt/cmake-3.29/bin/cmake .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig
# Install Trossen robot arm software (For AMD64 architectures, not Raspberry Pi) (This step may take up to 15 minutes)
#RUN sudo apt install curl
#RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
#RUN chmod +x xsarm_amd64_install.sh
#RUN ./xsarm_amd64_install.sh -d noetic -n
#RUN cp ./src/trossen/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d
#RUN cd ./src/trossen/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/ && \
#    service udev start && udevadm control --reload-rules && udevadm trigger
#RUN echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc && \
#    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc

# General rosdep install
RUN source /opt/ros/noetic/setup.bash \
    && apt-get update \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install -q -y \
      --from-paths ./src/ \
      --ignore-src \
      --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS workspace
RUN source /opt/ros/noetic/setup.bash \
    && catkin build

# Source the workspace setup files on container startup
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Configure display access (Unsets variable. Setting it may cause Rviz to fail.)
RUN echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc
# Disable hardware acceleration to reduce Rviz graphical issues
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
# Disable ROS1 EOL warnings
RUN echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> ~/.bashrc

# Install Trossen Interbotix software
# WORKDIR /root/catkin_ws/src/trossen
# RUN sudo apt install curl
# RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
# RUN chmod +x xsarm_amd64_install.sh
# RUN ./xsarm_amd64_install.sh -d noetic -n

# Restore workdir
WORKDIR /root/catkin_ws/

# Set udev rules
# COPY ./trossen/99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules
# RUN sudo service udev restart && sudo udevadm control --reload-rules && udevadm trigger
