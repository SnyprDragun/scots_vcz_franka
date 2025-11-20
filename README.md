# SCOTS-VCZ Aided Manipulator Motion using Franka Research 3 Robot

## Overview
Through this repo we simulate an example that demonstrates a novel control law for 2nd order Euler-Lagrange systems (Robotic Arm). We use SCoTs (Symbolic Control Toolbox) to generate controller for EE (end effector) pose (position + orientation) for a reach avoid task in the robot's reachable state space. We then use Virtual Confinement Zone (VCZ) control law to generate torque commands in robot's joint space, which formally gurantees adherence to EE trajectory prescribed by SCoTs.

## Requirements
* [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
* [SCoTs v0.2](https://github.com/mkhaled87/scots-ready)
* [franka_ros2](https://github.com/frankarobotics/franka_ros2)

## Methodology
1. First we generate formally verified controller using SCoTs, for some reach avoid example.
2. Then we create a reference trajectory with custom start point using matlab, and store it in a csv file.
3. Next we interpolate the csv file to 20000 or more way points for smoothening the discrete state space transitions of SCoTs controller.
4. Finally we feed these way points to the robot as desired end effector pose.
5. The VCZ control law performs a transformation from task space to joint space, prescribing desired joint torques that allows EE to track reference trajectory closely.
6. We publish these joint torques to robot for gazebo simulation.

## Setup
* Make sure you satisfy the requirements mentioned above. **SCoTs** and **franka_ros2** packages must be in separate folders.
* Run `franka.cc file` -> run `franka.m file` -> run `interpolate.py` file -> launch `gazebo_scots_vcz_example_controller.launch.py`
---
>***NOTE:*** To replicate results, follow this [documentation](https://github.com/SnyprDragun/scots_vcz_franka/blob/main/Documentation.md).
