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
- Run the command `docker run --rm -it --gpus all --network=host --privileged zed_bridge` to start the container.

#### Bare metal
- Install ROS Humble, if not already installed.
- Install CUDA, if not already installed.
- Install ZED SDK.
- Create a workspace and put this package in it.
- From the root of the workspace, run the command `colcon build --symlink-install --parallel-workers 8`.
