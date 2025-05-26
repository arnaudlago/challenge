# Challenge

## Author
Arnaud Lago

## Requirements

This challenge requires to develop a Path-Tracking Algorythm in C++ that have the capability to follow a specified path.
It should include a navigation state machine that showcases the current state of the controller.
The Path-Tracking Algorythm should be implemented as a ROS node and must run with the POLARIS_GEM_e2 simulator.

## Deliverables Overview

The deliverable is a zip folder with the following structure

    challenge
    |- Source

**Source**: source code of the project.

    challenge
    |- src
        |- challenge_pkg
            |- action -> ROS action message definition
            |- include -> contains header files
            |- launch -> ROS launch file to run the ROS node delivered
            |- lib -> Path-Tracking algorythm library
            |- paths -> csv files representing different paths
            |- src -> ROS nodes 
            |- srv -> ROS service message definition
            |- tests -> test files
        |- CMakeLists.txt
        |- package.xml
    |- Dockerfile



## Path Tracking Application

### Algorithm selection

I first selected the path tracking algorythm, after some research on internet, my choice was to select the **Pure Pursuit** algorythm for the following reasons:
 - **Simplicity and Ease of Implementation**: easy to understand and implement, few parameters.
 - **Smooth path following**: handles curved paths by continuously adjusting the steering resulting in smooth turns and reduced oscillations
 - **Flexibility**: can handle various types of paths, including straight lines and curves and adapted to the POLARIS_GEM_e2 model (car type).
 - **Robustness**: elatively tolerant to small errors in localization or path definition.

### Algorithm implementation

The algorythm is implemented in the *pure_pursuit.cpp* library for better partability.

**Pseudo code**:
- Receives a Path.
- Checks if the path is valid.
- Finds the point of the trajectory that is closest to the vehicle's reference pose.
- Computes steeing angle and speed to apply.

**Design considerations**:
- A path is valid if:
    - it contains at least two points,
    - the absolute value of the angle between 2 consecutive segments is inferior to PI/4.
- The speed control is only based on remaining distance to travel to stop smoothly.

**Nice to add**:
- Lateral error check to monitor if the vehicle is too far from the trajectory.
- PID for speed control based on trajectory curvature.

### ROS node implementation

#### Path Tracking Controller

The ROS node wraps the *pure_pursuit* library and implements a simple state machine.
It implements the following ROS features:

**service**
- *"/path_tracking_controller/set_path"*: to receive a [nav_msgs::Path](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)

**Action server**
- *Pause action*: This is to receive a PAUSE request from an external node. I choose to use an action since the response of the request may not be immediate because it takes time for the vehicle to properly pause.
    - **Note**: PAUSE action message is used to send a Pause AND a Track request.

**Subscribe**
- */odometry*: to receive the [nav_msgs::Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) from a vehicle or a simulation.

**Advertise**
- */ackermann_cmd*: to advertise an [ackermann_msgs::AckermannDrive](http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDrive.html) to a vehicle or a simulation.
- */path_tracking_controller/state*: to advertise a *string* representing the state of the node:
    - **IDLE**: no path to track
    - **PAUSED**: a path is loaded but vehicle is not tracking it.
    - **TRACKING**: vehicle is tracking the trajectory.
    - **ERROR**: Tracking error occured (not used) 
    - **PAUSING**: vehicle slowing down to pause due to PAUSE action.
        - **Note**: if the vehicle reaches the end of the trajectory while PAUSING, the Pause action request is failed by design.


#### Path Sender

This is a simple node use to send a trajectory to the **Path Tracking Controller**.

**Pseudo code**:
- send a trajectory
- send a PAUSE action message with **False** parameter to  make the vehicle follows the trajectory.
- wait 10 seconds
- send a PAUSE action message with **True** parameter to  make the vehicle stops the trajectory.
- wait 5 seconds after the vehicle is stopped.
- send a Track message (the vehicle follows the trajectory until it reaches the end).

## BUILD

`git clone https://github.com/arnaudlago/challenge.git`

`cd challenge`

`docker build -t challenge .`

## RUN

## RUN the simulation

It is necessary to run the [POLARIS_GEM_e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) simulator.
Follow the instructions to install the simulator and run gem_gazebo_rviz.launch and gem_sensor_info.launch.
You can use the following launch file:

```
<?xml version="1.0"?>
<launch>
    <arg name="robot_name" value="gem"/>
    <include file="$(find gem_gazebo)/launch/gem_gazebo_rviz.launch">
        <arg name="velodyne_points" default="true"/>
    </include>
    <include file="$(find gem_gazebo)/launch/gem_sensor_info.launch">
        <arg name="robot_name" value="$(arg robot_name)" />
    </include>
</launch>
```

## RUN the docker


`docker run -it --network=host challenge`





