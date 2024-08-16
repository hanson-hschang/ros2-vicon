# ROS2 Vicon Docker Image

## Overview
This Docker image provides a pre-configured environment for ROS2 (Robot Operating System 2) with Vicon motion capture system integration. 
It's built on top of the official ROS2 Foxy desktop image and includes additional tools and libraries for working with Vicon systems.

## Features
- Based on ROS2 Foxy
- Includes Vicon SDK and necessary dependencies
- Pre-configured for seamless integration between ROS2 and Vicon systems
- Suitable for robotics research and development involving motion capture

## Tags
- `YYYYMMDD`: Date-based tags for specific versions
- `latest`: Always points to the most recent build

## Usage
To pull and run this image:

```bash
docker pull hansonhschang/ros2-vicon:latest
docker run -it --rm --entrypoint bash hansonhschang/ros2-vicon:latest
```

For GUI applications, you may need to set up X11 forwarding or use other methods to enable GUI support.

## Building from Source
If you want to build the image yourself:

```bash
git clone https://github.com/hanson-hschang/ros2-vicon.git
cd ros2-vicon
docker build . -t ros2-vicon
```

## Contributing
Contributions to improve this Docker image are welcome. Please submit issues and pull requests on our [GitHub repository](https://github.com/hanson-hschang/ros2-vicon).

