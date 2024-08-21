# Use the official ROS 2 Foxy base image
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    cmake \
    wget \
    git \
    htop \
    vim \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get install -y \
    ros-foxy-ros2bag \
    ros-foxy-rosbag2-converter-default-plugins \
    ros-foxy-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /bag_files

# Install python packages
RUN pip3 install --upgrade pip
RUN pip3 install numpy matplotlib scipy scikit-learn
RUN pip3 install h5py torchvision==0.16.2 torch==2.1.2+cpu -f https://download.pytorch.org/whl/torch_stable.html
RUN rm -rf /root/.cache/pip

# Clone the ros2-vicon-receiver package
ENV WS /vicon_ws
RUN mkdir -p $WS/src && \
    cd $WS/src && \
    git clone https://github.com/skim0119/ros2-vicon-receiver && \
    cd ros2-vicon-receiver && \
    ./install_libs.sh

# Build the package
WORKDIR $WS
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

WORKDIR /
RUN source vicon_ws/install/setup.bash

RUN echo ". /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo ". /vicon_ws/install/setup.bash" >> /root/.bashrc

# Set alias
RUN echo "alias python=python3" >> /root/.bashrc
RUN echo "alias pip=pip3" >> /root/.bashrc

RUN sed -e '/[ -z "$PS1" ] && return/s/^/#/g' -i /root/.bashrc

# Set the entrypoint to use ROS 2
ENTRYPOINT ["/bin/bash", "-c", "source ~/.bashrc && exec \"$@\"", "--"]
CMD ["bash"]
