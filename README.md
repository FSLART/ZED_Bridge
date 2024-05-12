# ZED_Bridge
ROS bridge for the ZED 2i camera, since the official shitty one doesn't work.

## Setup

### Requirements
- ROS Humble
- ZED SDK
- 

### Instructions

#### Docker (recommended for development)
- Run the command `docker build -t zed_bridge .` to build the image.
- Run the command `docker run  -it --gpus all zed_bridge` to start the container. Inside the container, go to the `/catkin_ws` directory and make the following:
    - `source /opt/ros/humble/setup.bash`
    - `colcon build --symlink-install --parallel-workers 4`
- Exit the container and check its ID with `docker ps --all`.
- Run the command `docker commit --change 'CMD /bin/bash -c "source /opt/ros/humble/setup.bash /&& source /catkin_ws/install/setup.bash && ros2 run zed_bridge zed_bridge"' <my-container-id> zed_bridge`.
- Now, finally, run the command `docker run --rm -it --gpus all --network=host zed_bridge`.

#### Bare metal
- Install ROS Humble, if not already installed.
- Install CUDA, if not already installed.
- Install ZED SDK.
- Create a workspace and put this package in it.
- From the root of the workspace, run the command `colcon build --symlink-install --parallel-workers 8`.
- Run the command `ros2 run zed_bridge zed_bridge`.
