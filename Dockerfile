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
    && rm -rf /var/lib/apt/lists/*


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

RUN sed -e '/[ -z "$PS1" ] && return/s/^/#/g' -i /root/.bashrc

# Set the entrypoint to use ROS 2
ENTRYPOINT ["/bin/bash", "-c", "source ~/.bashrc && exec \"$@\"", "--"]
CMD ["bash"]