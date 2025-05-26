# Use an official ROS distribution as the base image
FROM ros:noetic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    ros-noetic-catkin \
    ros-noetic-ros-base \
    ros-noetic-nav-msgs \
    ros-noetic-ackermann-msgs \
    ros-noetic-tf \
    ros-noetic-actionlib \
    ros-noetic-geometry2 \
    && rm -rf /var/lib/apt/lists/*


# Create and set the working directory
#WORKDIR /catkin_ws/src
WORKDIR /challenge

# Copy your ROS package into the Docker container
ADD ./src /challenge/src

WORKDIR /challenge
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the setup script and start a ROS node
CMD ["/bin/bash", "-c", "source /challenge/devel/setup.bash && roslaunch src/challenge_pkg/launch/challenge.launch"]

