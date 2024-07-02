FROM fslart/cuda-ros:humble

# setup environment variables
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
ENV PATH=/usr/local/cuda/bin:$PATH

# Update the system
RUN apt-get update && apt-get upgrade -y

RUN echo "hello"
# Install ZED SDK
RUN apt-get install -y --no-install-recommends \
    wget \
    unzip \
    zstd
RUN wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O zed_sdk_v4.1.2.run
RUN chmod +x zed_sdk_v4.1.2.run
RUN ./zed_sdk_v4.1.2.run -- silent skip_cuda skip_tools
RUN rm zed_sdk_v4.1.2.run

# Update the system and install the CUDA toolkit
ENV NVIDIA_DRIVER_CAPABILITIES video,compute,utility

RUN apt-get update && apt upgrade -y

# Install the dependencies
RUN apt-get install -y ros-humble-sensor-msgs libopencv-dev

# Create the workspace
RUN mkdir -p /catkin_ws/src

# Clone lart_msgs
WORKDIR /catkin_ws/src
RUN git clone -b dev https://github.com/FSLART/lart_msgs.git

# install colcon and dependencies
RUN apt-get install -y python3-colcon-common-extensions python3-vcstool

# Install foxglove bridge
RUN apt-get install -y ros-humble-foxglove-bridge

# Copy the package to the container
COPY . /catkin_ws/src/zed_bridge

# Copy the calibration file to the container
COPY SN39866630.conf /usr/local/zed/settings/SN39866630.conf

WORKDIR /catkin_ws

# Build the package
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /catkin_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Run the package
# CMD /bin/bash -c "source /catkin_ws/devel/setup.bash && ros2 run zed_bridge zed_bridge"
