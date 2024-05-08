FROM fslart/cuda-ros:humble

# Install ZED SDK
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    unzip
RUN wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22 -O zed_sdk_v4.1.0.run
RUN chmod +x zed_sdk_v4.1.0.run
RUN ./zed_sdk_v4.1.0.run --silent

# Create the workspace
RUN mkdir -p /catkin_ws/src

# Copy the package to the container
COPY . /catkin_ws/src/zed_bridge

# Build the package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /catkin_ws && catkin_make"

# Run the package
CMD /bin/bash -c "source /catkin_ws/devel/setup.bash && ros2 run zed_bridge zed_bridge"