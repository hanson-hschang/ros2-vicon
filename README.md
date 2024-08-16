
[![Publish Docker Image to Docker Hub CI](https://github.com/hanson-hschang/ros2-vicon/actions/workflows/docker-image.yml/badge.svg?branch=main)](https://github.com/hanson-hschang/ros2-vicon/actions/workflows/docker-image.yml)

1. Download Docker from its [website](https://www.docker.com/)

2. Clone this repostitory 
```zsh
git clone https://github.com/hanson-hschang/ros2-vicon.git
```

3. Change directory to your downloaded folder
```zsh
cd ros2-vicon
```

4. Build the image from the `Dockerfile`. 
The `.` after `build` means to look for the `Dockerfile` in the current directory. 
The `-t <name>` means to tag the image with a name `<name>`. 
Note that the tag name can be any of your choice but it can only be in lower cases.
```zsh
docker build . -t <name>
```
> A built image on [Docker Hub](https://hub.docker.com/r/hansonhschang/ros2-vicon) may also be pulled directly.

5. To create and start a new container from the image, run the following command
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

6. Now you have entered the bash in the container just created. To exit, run the following command, and the container will be clean up because of the `--rm` flag in the previous command.
```bash
exit
```

7. (Optional) If one would like to keep the container running in the background after exiting, remove the flag `--rm` from the command in step 5.

8. (Optional) To check what containers are still running in the background, run
```zsh
docker ps --all  
```
Sample output:
```zsh
CONTAINER ID   IMAGE        COMMAND   CREATED          STATUS                     PORTS     NAMES
163d0594b46d   ros2-vicon   "bash"    13 seconds ago   Exited (0) 7 seconds ago             nice_margulis
```
In this case, a container with name `nice_margulis` is running.

9. (Optional) To re-start one container with name `<name>`
```zsh
docker start -i -a <name>
```
|  flag  | function  |
|  ------------ | ----------- |
|  `-i`         | keeps `STDIN` open even if not attached, allowing for interactive use  |
|  `-a`         | attaches the terminal to the container's `STDOUT` (standard output) and `STDERR` (standard error) streams. This means you'll see the output from the container's main process in your terminal. |

10. (Optional) To remove one container with name `<name>`
```zsh
docker rm <name>
```

11. (Optional) To remove an image with name `<name>`
```zsh
docker rmi <name>
```

> All these commands have a GUI in the Docker Desktop app as well.
