FROM nvidia/cuda:13.3.1-cudnn-devel-ubuntu24.04 AS base
# RUN rm /etc/apt/sources.list.d/nvidia-ml.list && apt clean && apt update

# Use bash as shell for RUN commands, and use --login to ensure conda loads once installed
SHELL ["/bin/bash", "-c"]

# Set the ROS distribution to use
ARG ROS_DISTRO=jazzy
ARG DEBIAN_FRONTEND=noninteractive

# Install tools needed to configure the ROS repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 repository
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       > /etc/apt/sources.list.d/ros2.list

# Install and upgrade system, ROS, and development packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        apt-utils \
        build-essential bc bzip2 wget gawk flex bison dwarves linux-lowlatency \
        git pipx udev \
        locales lsb-release \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-rqt-controller-manager \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-pinocchio \
        python3-rosdep python3-vcstool \
        libegl1-mesa-dev libelf-dev libglfw3 libglew2.2 \
        libgl1 libglx-mesa0 libnet1-dev libosmesa6 libspnav-dev libssl-dev \
        mesa-utils mesa-va-drivers vainfo screen x11-apps \
        lsof psmisc spacenavd xterm gdb \
        python3-full python-is-python3 python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && dpkg-reconfigure locales \
    && rosdep init \
    && rosdep fix-permissions

# FROM base AS dev

# Set the working directory in the container
WORKDIR /root/ros2_ws/

# Copy the morpheus repo
COPY ./ ./src/

# Source ROS setup files on interactive terminal startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# General rosdep install (Includes EOL distros to handle Interbotix library dependencies)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt update \
    && rosdep update --include-eol-distros \
    && rosdep install -q -y -r \
      --from-paths ./src/ \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# # Build the ROS workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

# Source the workspace setup files on container startup
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Create and activate venv to avoid conflict with externally managed environment
ENV VIRTUAL_ENV=.venv
RUN python3 -m venv ${VIRTUAL_ENV} --system-site-packages --symlinks
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"
ENV PYTHONPATH="${VIRTUAL_ENV}/lib/python3.12/site-packages:${PYTHONPATH}"
RUN echo "source /root/ros2_ws/${VIRTUAL_ENV}/bin/activate" >> ~/.bashrc

# # Install non-debian python dependencies 
# # (Force pip to permit package installation)
# RUN echo "[global]" >> etc/pip.conf
# RUN echo "break-system-packages = true" >> etc/pip.conf
RUN ${VIRTUAL_ENV}/bin/python3 -m pip install --upgrade pip
RUN ${VIRTUAL_ENV}/bin/python3 -m pip install --no-cache-dir -r ./src/requirements.txt

# Install GELLO dependencies
RUN ${VIRTUAL_ENV}/bin/python3 -m pip install --no-cache-dir -r ./src/submodules/gello_software/requirements.txt
RUN ${VIRTUAL_ENV}/bin/python3 -m pip install --no-cache-dir -e ./src/submodules/gello_software/. --use-pep517
RUN ${VIRTUAL_ENV}/bin/python3 -m pip install --no-cache-dir -e ./src/submodules/gello_software/third_party/DynamixelSDK/python/.

# Configure display access (Unsets variable. Setting it may cause Rviz to fail.)
# RUN echo "unset LIBGL_ALWAYS_INDIRECT" >> ~/.bashrc
# Disable hardware acceleration to reduce Rviz graphical issues
# RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

# Install Trossen Interbotix software
# WORKDIR /root/ros2_ws/src/trossen
# RUN sudo apt install curl
# RUN curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
# RUN chmod +x xsarm_amd64_install.sh
# RUN ./xsarm_amd64_install.sh -d $ROS_DISTRO -n
# Restore workdir
# WORKDIR /root/ros2_ws/

# Set udev rules
# COPY ./trossen/99-interbotix-udev.rules /etc/udev/rules.d/99-interbotix-udev.rules
# RUN sudo service udev restart && sudo udevadm control --reload-rules && udevadm trigger
