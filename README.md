# Docker of ROS2 with Vicon

[![Publish Docker Image to Docker Hub CI](https://github.com/hanson-hschang/ros2-vicon/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/hanson-hschang/ros2-vicon/actions/workflows/docker-image.yml)

This repository contains a Dockerfile and associated scripts for creating a Docker image that combines ROS2 (Robot Operating System 2) with Vicon motion capture system integration.

## Overview

This Docker image is designed to provide a pre-configured environment for robotics research and development, particularly for projects that involve both ROS2 and Vicon motion capture systems. It's based on the official ROS2 Foxy desktop image and includes additional tools and libraries for working with Vicon systems.

## Features

- ROS2 Foxy desktop installation
- Vicon SDK and necessary dependencies
- Pre-configured for seamless integration between ROS2 and Vicon systems
- Suitable for robotics research and development involving motion capture

## Prerequisites

- [Docker](https://www.docker.com/) installed on your system
- Git (for cloning this repository)

## Download the Docker Image

### Method 1: Pull from Docker Hub

Docker image is built using `github-action`, and uploaded to [Docker Hub](https://hub.docker.com/r/hansonhschang/ros2-vicon).

```zsh
docker pull hansonhschang/ros2-vicon:latest
```

### Method 2: Build locally from Dockrfile

To build the Docker image locally:

1. Clone this repostitory 
  ```zsh
  git clone https://github.com/hanson-hschang/ros2-vicon.git
  ```

2. Change directory to your downloaded folder
  ```zsh
  cd ros2-vicon
  ```

3. Build the image from the `Dockerfile`. 
  The `.` after `build` means to look for the `Dockerfile` in the current directory. 
  The `-t <name>` means to tag the image with a name `<name>`. 
  Note that the tag name can be any of your choice but it can only be in lower cases.
  ```zsh
  docker build . -t <name>
  ```
  > A built image on [Docker Hub](https://hub.docker.com/r/hansonhschang/ros2-vicon) may also be pulled directly.

## Using the Docker Image

1. To create and start a new container from the image, run the following command
  ```zsh
  docker run -i -t --rm --entrypoint bash <name>
  ```
  |  flag&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  |  value&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | function  |
  |  ------------  | ------------ | ----------- |
  |  `-i`          |              | keeps `STDIN` open even if not attached, allowing for interactive use  |
  |  `-t`          |              | allocates a pseudo-TTY, which provides an interactive shell in the container |
  | `--rm`         |              | automatically removes the container when it exits. It's useful for creating temporary containers that clean up after themselves |
  | `--entrypoint` | `bash`       | overrides the default entrypoint of the container with bash. It means that instead of running whatever command was specified as the default in the `Dockerfile`, the container will start a bash shell. |
  |                | `<name>` | name of the Docker image to use for creating the container.

2. Now you have entered the bash in the container just created. To exit, run the following command, and the container will be clean up because of the `--rm` flag in the previous command.
  ```bash
  exit
  ```

3. (Optional) If one would like to keep the container running in the background after exiting, remove the flag `--rm` from the command in step 1.

4. (Optional) To check what containers are still running in the background, run
  ```zsh
  docker ps --all  
  ```
  Sample output:
  ```zsh
  CONTAINER ID   IMAGE        COMMAND   CREATED          STATUS                     PORTS     NAMES
  163d0594b46d   ros2-vicon   "bash"    13 seconds ago   Exited (0) 7 seconds ago             nice_margulis
  ```
  In this case, a container with name `nice_margulis` is running.

5. (Optional) To re-start one container with name `<name>`
  ```zsh
  docker start -i -a <name>
  ```
  |  flag  | function  |
  |  ------------ | ----------- |
  |  `-i`         | keeps `STDIN` open even if not attached, allowing for interactive use  |
  |  `-a`         | attaches the terminal to the container's `STDOUT` (standard output) and `STDERR` (standard error) streams. This means you'll see the output from the container's main process in your terminal. |

6. (Optional) To remove one container with name `<name>`
  ```zsh
  docker rm <name>
  ```

7. (Optional) To remove an image with name `<name>`
  ```zsh
  docker rmi <name>
  ```

> All these commands have a GUI in the Docker Desktop app as well.

For GUI applications, you may need to set up X11 forwarding or use other methods to enable GUI support.

## CI/CD

This repository uses GitHub Actions for continuous integration and delivery. On each pull request to the main branch, the workflow will:

1. Build the Docker image
2. Push the image to Docker Hub with a date-based tag and 'latest' tag

The workflow can also be manually triggered from the GitHub Actions tab.

## Contributing

Contributions to improve this Docker image are welcome. Please submit issues and pull requests on this GitHub repository.
