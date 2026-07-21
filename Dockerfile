FROM nvidia/cuda:13.1.0-base-ubuntu24.04 AS base
# RUN rm /etc/apt/sources.list.d/nvidia-ml.list && apt clean && apt update

# Use bash as shell for RUN commands, and use --login to ensure conda loads once installed
SHELL ["/bin/bash", "-c"]

# Ensure apt is up to date
RUN apt update && apt upgrade --no-install-recommends -y

# Install setup dependencies
RUN apt update && apt install --no-install-recommends -y \
    apt-utils \
    build-essential \
    bc \
    ca-certificates \
    gnupg2 \
    bzip2 \
    libssl-dev \
    wget \
    gawk \
    flex \
    bison \
    libelf-dev \
    dwarves \
    linux-lowlatency \
    curl \
    git \
    pipx \
    x11-apps \
    mesa-utils \
    udev \
    && rm -rf /var/lib/apt/lists/*

# # Set up realtime kernel to ensure smooth robot operation
# https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/real_time.rst
# RUN mkdir -p ${HOME}/rt_kernel_build
# WORKDIR ${HOME}/rt_kernel_build
# # Select a realtime kernel version compatible with this Dockerfile's base image.
# # See top of file for base image. Current base kernel is: 6.6.87.2-microsoft-standard-WSL2
# # https://wiki.linuxfoundation.org/realtime/preempt_rt_versions
# RUN export KERNEL_MAJOR_VERSION=6
# RUN export KERNEL_MINOR_VERSION=6
# RUN export KERNEL_PATCH_VERSION=129
# RUN export RT_PATCH_VERSION=70
# RUN export KERNEL_VERSION="$KERNEL_MAJOR_VERSION.$KERNEL_MINOR_VERSION.$KERNEL_PATCH_VERSION"
# # Download the kernel sources, patch sources, and their signature files:
# RUN wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/$KERNEL_MAJOR_VERSION.$KERNEL_MINOR_VERSION/patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.xz
# RUN wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/$KERNEL_MAJOR_VERSION.$KERNEL_MINOR_VERSION/patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign
# RUN wget https://www.kernel.org/pub/linux/kernel/v$KERNEL_MAJOR_VERSION.x/linux-$KERNEL_VERSION.tar.xz
# RUN wget https://www.kernel.org/pub/linux/kernel/v$KERNEL_MAJOR_VERSION.x/linux-$KERNEL_VERSION.tar.sign
# # Unzip the downloaded files
# RUN xz -dk patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.xz
# RUN xz -d linux-$KERNEL_VERSION.tar.xz
# # Verify the downloads using the signature files and the kernel.org PGP keys
# RUN gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org
# RUN gpg2 --verify patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.sign
# RUN gpg2 --verify linux-$KERNEL_VERSION.tar.sign
# # Extract the kernel sources, apply the realtime patch, and build the kernel
# RUN tar xf linux-$KERNEL_VERSION.tar
# WORKDIR linux-$KERNEL_VERSION
# RUN xzcat ../patch-$KERNEL_VERSION-rt$RT_PATCH_VERSION.patch.xz | patch -p1
# RUN make oldconfig
# RUN scripts/config --disable SYSTEM_TRUSTED_KEYS
# RUN scripts/config --disable SYSTEM_REVOCATION_KEYS
# RUN make -j `getconf _NPROCESSORS_ONLN` deb-pkg
# RUN sudo apt install ../linux-headers-$KERNEL_VERSION-rt$RT_PATCH_VERSION*.deb \
#                    ../linux-image-$KERNEL_VERSION-rt$RT_PATCH_VERSION*.deb

# ROS 2 preinstall setup
ENV ROS_DISTRO=jazzy
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install --no-install-recommends -y \
    locales \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*
# Set locale
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN dpkg-reconfigure locales
# Enable ROS 2 apt repositories
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
# RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
# RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
# RUN dpkg -i /tmp/ros2-apt-source.deb
RUN apt update && apt install ros-dev-tools -y

# Install ROS ${ROS_DISTRO} Desktop
RUN apt update && apt upgrade -y
RUN apt update && apt install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup files on interactive terminal startup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install rosdep and related tools
RUN apt update && apt install -y --no-install-recommends \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*
# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions

# Install rosserial for network communications
# RUN apt update && apt install -y --no-install-recommends \
#     ros-${ROS_DISTRO}-rosserial \
#     ros-${ROS_DISTRO}-rosserial-python \
#     ros-${ROS_DISTRO}-rosserial-arduino \
#     && rm -rf /var/lib/apt/lists/*

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
RUN apt update && apt install --no-install-recommends -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-rqt-controller-manager \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
#     ros-${ROS_DISTRO}-teleop-twist-keyboard \
#     python3-tk \
    && rm -rf /var/lib/apt/lists/*

# # Install general dependencies
RUN apt update && apt install --no-install-recommends -y \
    libnet1-dev \
#     libcxx-serial-dev \
    libspnav-dev \
    spacenavd \
    screen \
    && rm -rf /var/lib/apt/lists/*

# # Install debian python dependencies 
RUN apt update && apt install --no-install-recommends -y \
    python3-full \
    python-is-python3 \
    python3-pip \
    python3-venv \
    python3-zipp \
    python3-pymodbus \
    python3-numpy \
    python3-scipy \
    python3-pynput \
    python3-pygame \
    python3-importlib-metadata \
    python3-six \
    python3-setuptools \
    python3-pyqt6 \
    && rm -rf /var/lib/apt/lists/*

# # Install non-debian python dependencies 
# # (Force pip to permit package installation)
RUN echo "[global]" >> etc/pip.conf
RUN echo "break-system-packages = true" >> etc/pip.conf
# # (Relatively error-prone dependencies installed individually for readability of error messages)
RUN python3 -m pip install numpy-quaternion
RUN python3 -m pip install readchar
RUN python3 -m pip install PyQt6
RUN python3 -m pip install transforms3d
RUN python3 -m pip install modern_robotics
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

# # Install GELLO dependencies
# RUN pip install -r ./src/gello_software/requirements.txt
# RUN pip install -e ./src/gello_software/. --use-pep517
# RUN pip install -e ./src/gello_software/third_party/DynamixelSDK/python/.
# RUN pip install pylsl

# # Build liblsl from source (for pylsl)
# RUN apt update && apt install -y \
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

# Clean & upgrade
RUN apt update && apt dist-upgrade

# FROM base AS dev

# Set the working directory in the container
WORKDIR /root/ros2_ws/

# Copy the morpheus repo
COPY ./ ./src/

# General rosdep install
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && apt update \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install -q -y \
      --from-paths ./src/ \
      --ignore-src \
      --rosdistro $ROS_DISTRO \
    && rm -rf /var/lib/apt/lists/*

# # Build the ROS workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install

# Source the workspace setup files on container startup
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

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
