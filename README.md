# Development of a Modular Anthropomorphic Robotic Manipulator

## Introduction

This project focuses on the development of a cost-effective, modular, anthropomorphic robotic manipulator with 'n'-Degrees-of-Freedom (DoF). The primary objective is to create a flexible system that can be easily customized and assembled using 3D printable links. These links allow for various configurations, providing a versatile platform for robotic manipulation.

The software architecture of this system is designed to be reconfigurable, supporting automated generation of necessary description and configuration files. This capability facilitates seamless visualization, planning, and control of the custom configurations. The Robot Operating System (ROS) is utilized as a digital twin, enabling efficient simulation and control of the manipulator setups.

Hardware modules are developed to accompany each link, enabling independent joint control regardless of the motor type used. Communication between the hardware and the ROS software is established via a CAN-based OpenCyPhaL network. The system's performance is validated through experimental demonstrations of specific configurations and custom hardware assemblies.

## Repository Contents

This repository contains the following components:

### docker_image

- **docker-compose file**: Contains a Docker Compose configuration file to run a pre-configured Humble-ROS container. 
### ws_custom_robot

- **ROS Packages**: Collection of all developed ROS (Robot Operating System) packages, these packages include modules for visualization, planning, control, and communication with the hardware modules.

### firmware

- **Firmware**: Includes all implemented firmware for the hardware modules accompanying each link of the manipulator. 

### Prerequisites

* Setup [Docker] (https://docs.docker.com/engine/install/ubuntu/)
* Download [Docker compose file] (https://github.com/josemartins2121/modular_arm_ros2/tree/main/docker_image)
* Go the save folder as docker compose is saved 
* Initialize a humble-desktop Docker container
```sh
DOCKER_IMAGE=humble-desktop docker compose run --name custom_name cpu

```
* Install ROS 2 missing libraries
```sh
sudo apt-get update && sudo apt-get install -y \
     ros-humble-joint-state-publisher-gui \
     ros-humble-gazebo-ros \
     ros-humble-xacro \
     ros-humble-ros2-control \
     ros-humble-moveit \
     ros-humble-ros2-controller \
     ros-humble-gazebo-ros2-control 
```
* Install  C++ additional libraries
```sh
sudo apt-get update && sudo apt-get install -y \
     libserial-dev \
```

### Installation

1. Clone the repo
```sh
git clone [https://github.com/josemartins2121/modular_arm_ros2.git]
```
2. Build the ROS 2 workspace
```sh
cd ~/ws_custom_robot
colcon build
```
3. Source the project
```sh
. install/setup.bash
```


## Usage

To launch Rviz, to visualize the current manipulator configuration
```sh
ros2 launch custom_robot_description display.launch.py
```

To launch the robot simulation and control, connect the Serial Node to the PC 
```sh
ros2 launch custom_robot_bringup custom_robot_bringup.launch.py
```


