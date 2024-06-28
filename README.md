# Development of a Modular Anthropomorphic Robotic Manipulator

## Introduction

This project focuses on the development of a cost-effective, modular, anthropomorphic robotic manipulator with 'n'-Degrees-of-Freedom (DoF). The primary objective is to create a flexible system that can be easily customized and assembled using 3D printable links. These links allow for various configurations, providing a versatile platform for robotic manipulation.

The software architecture of this system is designed to be reconfigurable, supporting automated generation of necessary description and configuration files. This capability facilitates seamless visualization, planning, and control of the custom configurations. The Robot Operating System (ROS) is utilized as a digital twin, enabling efficient simulation and control of the manipulator setups.

Hardware modules are developed to accompany each link, enabling independent joint control regardless of the motor type used. Communication between the hardware and the ROS software is established via a CAN-based OpenCyPhaL network. The system's performance is validated through experimental demonstrations of specific configurations and custom hardware assemblies.

## Repository Contents

This repository contains the following components:

### docker_image

- **docker-compose file**: Contains a Docker Compose configuration file to run a pre-configured Humble-ROS container. This container is set up with appropriate configurations to support the robotic manipulator project.

### ws_custom_robot

- **ROS Packages**: Collection of all developed ROS (Robot Operating System) packages specific to the anthropomorphic robotic manipulator project. These packages include modules for visualization, planning, control, and communication with the hardware modules.

### firmware

- **Firmware**: Includes all implemented firmware for the hardware modules accompanying each 3D printable link of the manipulator. The firmware enables independent joint control and communication with the ROS software via a CAN-based OpenCyPhaL network.

Each section of the repository plays a crucial role in the development, simulation, and validation of the modular anthropomorphic robotic manipulator system.
