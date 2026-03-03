FROM nvidia/cuda:13.1.0-base-ubuntu24.04 AS base
# RUN rm /etc/apt/sources.list.d/nvidia-ml.list && apt-get clean && apt-get update

# Use bash as shell for RUN commands, and use --login to ensure conda loads once installed
SHELL ["/bin/bash", "--login", "-c"]

# Ensure apt-get is up to date
RUN apt-get update && apt-get upgrade --no-install-recommends -y

# Install basic dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    wget \
    udev \
    ca-certificates \
    bzip2 \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Minimal ROS setup
ENV ROS_DISTRO=jazzy
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install --no-install-recommends -y \
    locales \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*
RUN dpkg-reconfigure locales
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
RUN apt-get update && apt-get install ros-dev-tools -y

# Install ROS ${ROS_DISTRO} Desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup files on interactive terminal startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install rosdep and related tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-vcstools \
    build-essential \
    && rm -rf /var/lib/apt/lists/*
# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update --rosdistro $ROS_DISTRO

# Install rosserial for network communications
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS_DISTRO}-rosserial \
#     ros-${ROS_DISTRO}-rosserial-python \
#     ros-${ROS_DISTRO}-rosserial-arduino \
#     && rm -rf /var/lib/apt/lists/*

FROM base AS dev

# Set the working directory in the container
WORKDIR /root/ros2_ws/

# Copy the morpheus repo
COPY ./ ./src/

# General rosdep install
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && apt-get update \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install -q -y \
      --from-paths ./src/ \
      --ignore-src \
      --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

# # Install Miniconda
# RUN wget --progress=dot:giga \
#       https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
#     bash /tmp/miniconda.sh -b -p /opt/conda && \
#     rm /tmp/miniconda.sh && \
#     /opt/conda/bin/conda clean -afy
# RUN mkdir -p ~/miniconda3
# RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
# RUN bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
# RUN rm ~/miniconda3/miniconda.sh
# ENV CONDA_PLUGINS_AUTO_ACCEPT_TOS=true
# ENV PATH=/opt/miniconda3/bin:$PATH
# # RUN echo "source ~/miniconda3/bin/activate" >> ~/.bashrc
# RUN source ~/miniconda3/bin/activate \
#     && conda update -n base -c defaults conda \
#     && conda init --all \
#     && conda config --set auto_activate_base true
# # Create virtual environment to handle dependencies
# COPY environment.yml .
# RUN echo "conda activate" >> ~/.bashrc
# #RUN conda env create -f environment.yml

# # Install ROS dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     ros-${ROS_DISTRO}-moveit \
#     ros-${ROS_DISTRO}-teleop-twist-keyboard \
#     python3-tk \
#     && rm -rf /var/lib/apt/lists/*

# # Install general dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     python3-pip \
#     python3-venv \
#     python3-zipp \
#     python-is-python3 \
#     libspnav-dev \
#     spacenavd \
#     && rm -rf /var/lib/apt/lists/*

# # Install python dependencies 
# # (Relatively error-prone dependencies installed individually for readability of error messages)
# RUN pip install --upgrade pip
# RUN pip install --upgrade \ 
#     pyserial \
#     pymodbus===2.1.0 \
#     numpy \
#     numpy-quaternion \
#     scipy \
#     readchar \
#     pynput \
#     pygame-ce
# RUN pip install --upgrade importlib_metadata
# RUN pip install --upgrade six
# RUN pip install --upgrade setuptools
# RUN pip install --upgrade PyQt6
# RUN pip install --upgrade modern_robotics

# # Install GELLO dependencies
# RUN pip install -r ./src/gello_software/requirements.txt
# RUN pip install -e ./src/gello_software/. --use-pep517
# RUN pip install -e ./src/gello_software/third_party/DynamixelSDK/python/.
# RUN pip install pylsl

# # Build liblsl from source (for pylsl)
# RUN apt-get update && apt-get install -y \
#       git cmake g++ libpugixml-dev && \
#     rm -rf /var/lib/apt/lists/*

# # Clone and build liblsl
# RUN cd /root && \
#     git clone https://github.com/sccn/liblsl.git && \
#     cd liblsl && \
#     mkdir build && cd build && \
#     /opt/cmake-3.29/bin/cmake .. && \
#     make -j"$(nproc)" && \
#     make install && \
#     ldconfig

# Install Trossen robot arm software (For AMD64 architectures, not Raspberry Pi) (This step may take up to 15 minutes)
#RUN sudo apt install curl
#RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
#RUN chmod +x xsarm_amd64_install.sh
#RUN ./xsarm_amd64_install.sh -d $ROS_DISTRO -n
#RUN cp ./src/trossen/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d
#RUN cd ./src/trossen/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/ && \
#    service udev start && udevadm control --reload-rules && udevadm trigger
#RUN echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc && \
#    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc

# # Build the ROS workspace
# RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
#     && colcon build

# Source the workspace setup files on container startup
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Configure display access (Unsets variable. Setting it may cause Rviz to fail.)
RUN echo "export LIBGL_ALWAYS_INDIRECT=" >> ~/.bashrc
# Disable hardware acceleration to reduce Rviz graphical issues
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
# Disable ROS1 EOL warnings
RUN echo "export DISABLE_ROS1_EOL_WARNINGS=1" >> ~/.bashrc

# Install Trossen Interbotix software
# WORKDIR /root/ros2_ws/src/trossen
# RUN sudo apt install curl
# RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
# RUN chmod +x xsarm_amd64_install.sh
# RUN ./xsarm_amd64_install.sh -d $ROS_DISTRO -n

# Restore workdir
WORKDIR /root/ros2_ws/

# Set udev rules
# COPY ./trossen/99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules
# RUN sudo service udev restart && sudo udevadm control --reload-rules && udevadm trigger
