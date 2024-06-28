# Development of a Modular Anthropomorphic Robotic Manipulator

## Introduction

This project focuses on the development of a cost-effective, modular, anthropomorphic robotic manipulator with 'n'-Degrees-of-Freedom (DoF). The primary objective is to create a flexible system that can be easily customized and assembled using 3D printable links. These links allow for various configurations, providing a versatile platform for robotic manipulation.

The software architecture of this system is designed to be reconfigurable, supporting automated generation of necessary description and configuration files. This capability facilitates seamless visualization, planning, and control of the custom configurations. The Robot Operating System (ROS) is utilized as a digital twin, enabling efficient simulation and control of the manipulator setups.

Hardware modules are developed to accompany each link, enabling independent joint control regardless of the motor type used. Communication between the hardware and the ROS software is established via a CAN-based OpenCyPhaL network. The system's performance is validated through experimental demonstrations of specific configurations and custom hardware assemblies.
