FROM thbarkouki/homestri-ur:minimal-v1.0.3

SHELL ["/bin/bash", "-c"]

WORKDIR /root/catkin_ws

# Install git
RUN apt-get update && apt-get install git -y

# # Install python dependencies
# RUN pip3 install \ 
#     package1 \
RUN pip3 install \
      numpy \
      numpy-quaternion \
      scipy 

RUN sudo apt-get update && sudo apt-get install python3-tk -y

# # Install ROS dependencies
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     package2 \
#     && rm -rf /var/lib/apt/lists/*
RUN apt-get install ros-noetic-teleop-twist-keyboard --no-install-recommends -y \
#     && moveit_visual_tools \
    && rm -rf /var/lib/apt/lists/*

COPY ./ ./src/morpheus

# General rosdep install (not necessary?)
RUN source /opt/ros/noetic/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install -q -y \
      --from-paths ./src \
      --ignore-src \
      --rosdistro noetic \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS workspace
RUN source /opt/ros/noetic/setup.bash
RUN catkin build -s
RUN source /root/catkin_ws/devel/setup.bash

# Installation for ros_pybullet_interface
# RUN git clone -b main https://github.com/ros-pybullet/ros_pybullet_interface.git /root/catkin_ws/src/ros_pybullet_interface
# RUN source /root/catkin_ws/devel/setup.bash
# RUN sudo apt update
# RUN cd /root/catkin_ws/src/morpheus_teleop/
# RUN bash install.sh 
# Might need to run catkin build -s -j1 to avoid crash

# ros_pybullet_interface citenote for BibTeX
# @article{Mower2022,
#   author = {Mower, Christopher E. and Stouraitis, Theodoros and Moura, João and Rauch, Christian and Yan, Lei and Behabadi, Nazanin Zamani and Gienger, Michael and Vercauteren, Tom and Bergeles, Christos and Vijayakumar, Sethu},
#   title = {ROS-PyBullet Interface: A Framework for Reliable Contact Simulation and Human-Robot Interaction},
#   journal = {[to appear] Proceedings of the Conference on Robot Learning},
#   year = {2022},
# }