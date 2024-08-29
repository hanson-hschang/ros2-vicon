# Docker of ROS2 with Vicon

[![Publish Docker Image to Docker Hub CI](https://github.com/hanson-hschang/ros2-vicon/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/hanson-hschang/ros2-vicon/actions/workflows/docker-image.yml)

This repository contains a Dockerfile and associated scripts for creating a Docker image that combines ROS2 (Robot Operating System 2) with Vicon motion capture system integration. The image includes this [ros2-node](https://github.com/OPT4SMART/ros2-vicon-receiver) to communicate with external Vicon system.

## Overview

This Docker image is designed to provide a pre-configured environment for robotics research and development, particularly for projects that involve both ROS2 and Vicon motion capture systems. It's based on the official ROS2 Foxy desktop image and includes additional tools and libraries for working with Vicon systems.

## Features

- ROS2 Foxy desktop installation
  - Python 3.8 (installed with ROS2 docker image)
- Vicon SDK and necessary dependencies
- Pre-configured for integration between ROS2 and Vicon systems
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

> The variable `<name>` in following sections will be `hansonhschang/ros2-vicon`.


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

To find more details on how to use Docker, please refer Docker's official documentation. Here, we list couple of CLI commands that can be useful to setup.

1. To create and start a new container from the image, run the following command
  ```zsh
  docker run -i -t --rm <name>
  docker run -i -t --rm --entrypoint bash <name>
  ```
  |  flag&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  |  value&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; | function  |
  |  ------------  | ------------ | ----------- |
  |  `-i`          |              | keeps `STDIN` open even if not attached, allowing for interactive use  |
  |  `-t`          |              | allocates a pseudo-TTY, which provides an interactive shell in the container |
  | `-v`           | `<host_path>:<container_path>` | Mount the host volume to container |
  | `--rm`         |              | automatically removes the container when it exits. It's useful for creating temporary containers that clean up after themselves |
  | `--entrypoint` | `bash`       | overrides the default entrypoint of the container with bash. It means that instead of running whatever command was specified as the default in the `Dockerfile`, the container will start a bash shell. |
  |                | `<name>` | name of the Docker image to use for creating the container.

 > One may also combine two tags together, e.g. `-i -t` is the same as `-it`. 

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
  In this case, a container of name: `nice_margulis` with container id: `163d0594b46d` is running.

5. (Optional) To re-start one container with name `<container_id>`
  ```zsh
  docker start -i -a <container_name>
  ```
  |  flag  | function  |
  |  ------------ | ----------- |
  |  `-i`         | keeps `STDIN` open even if not attached, allowing for interactive use  |
  |  `-a`         | attaches the terminal to the container's `STDOUT` (standard output) and `STDERR` (standard error) streams. This means you'll see the output from the container's main process in your terminal. |

6. (Optional) To remove one container with name `<container_id>`
  ```zsh
  docker rm <container_id>
  ```

7. (Optional) To remove an image with name `<name>`
  ```zsh
  docker rmi <name>
  ```

> All these commands have a GUI in the Docker Desktop app as well.

For GUI applications, you may need to set up X11 forwarding or use other methods to enable GUI support.

## Using the ROS2

You can run `ros2` commands as if you have `ros2` installed on local machine. Just prepend `docker run -it --rm <name> ros2`.

> To use your own script or folder outside of the container, you have to create the container without `--rm` flag, and copy the contents inside. Take a look [here](https://docs.docker.com/reference/cli/docker/container/cp/).

### Test Ros2: Talker and Listener

> The structure is similar to [this](https://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html#run-two-nodes-in-two-separate-docker-containers).

Run the following ros2 nodes on each terminal:

```zsh
docker run -it --rm <name> ros2 run demo_nodes_cpp talker
docker run -it --rm <name> ros2 run demo_nodes_cpp listener
```

### Launch Vicon

> The default host ip is `192.168.1.12`.

```zsh
docker run -it --rm <name> ros2 launch vicon_receiver client.launch.py
```

To see the topics, you may use 
```zsh
docker run -it --rm <name> ros2 topic list
```

To see the message in the topic `<topic>`, you can implement your own listener or use 
```zsh
docker run -it --rm <name> ros2 topic echo <topic>
```

#### Mock Vicon System

Mock system can be launched when Vicon system is not available.

```zsh
docker run -it --rm <name> ros2 launch vicon_receiver mock_client.launch.py
```


### Record using Ros2bag

More documentation can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

#### Save on host

Using the docker `-v` flag which mounts the host directory, data can be directly saved in host file system.

```zsh
docker run -it --rm \
  -v <host_path>:<container_path> \
  <name> \
  ros2 bag record -o <container_path>/<record_name> /topic1 /topic2 /topic3
```

#### Save on container

<details>
  <summary>Click me</summary>

The docker image includes the directory `/bag_files` for users to save the file. To use directory as your working directory for `ros2bag`, you will need to use two terminals. 

In the first one, create a container
```zsh
docker run -it --rm <name> 
```
and when you want to start the record, run
```bash
ros2 bag record -o bag_files/<record_name> /topic1 /topic2 /topic3
```
where `-o` tag means output file directory.

To finish the recording, press `control+c` directly.
This will stop the recording and save the data in the folder `<record_name>` directly under the `bag_files` directory.

To move the recording outside of the container, run the following command in the second terminal
```zsh
docker cp <container_id>:/bag_files/<record_name> /path/to/your/local/directory
```
and you are done!

To move the recording `<record_name>` from your local end into the container, run
```zsh
docker cp /path/to/your/local/directory/<record_name> <container_id>:/bag_files/
```

To replay the recording `<record_name>` in the container, run
```bash
ros2 bag play bag_files/<record_name>
```

> When you use `ros2bag`, make sure you download the saved `bag_files` before exiting or removing the docker container.

</details>

### Other Ros2 Tips

More commands are available in official ROS2 documentation. Here, I'm providing few that will be useful
to debug the `docker` container.

```zsh
docker run -it --rm <name> ros2 pkg list    # list all available ros2 packages
docker run -it --rm <name> ros2 topic list  # list all available ros2 topics
docker run -it --rm <name> ros2 topic echo <topic>  # listen to a topic data
```

## CI/CD

This repository uses GitHub Actions for continuous integration and delivery. On each pull request to the main branch, the workflow will:

1. Build the Docker image
2. Push the image to Docker Hub with a date-based tag and 'latest' tag

The workflow can also be manually triggered from the GitHub Actions tab.

## Contributing

Contributions to improve this Docker image are welcome. Please submit issues and pull requests on this GitHub repository.
