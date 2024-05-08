# ZED_Bridge
ROS bridge for the ZED 2i camera, since the official shitty one doesn't work.

## Setup

### Requirements
- ROS Humble
- ZED SDK

### Instructions

#### Docker (recommended for development)
- Run the command `docker build -t zed_bridge .` to build the image.
- Run the command `docker run --rm -it --gpus all --network=host zed_bridge` to start the container.

#### Bare metal
- Install ROS Humble, if not al
- Install ZED SDK
- 