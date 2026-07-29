# Delta Robot

A ROS 2-based delta robot for high-speed pick-and-place applications.

## 🎥 Demo

<p align="center">
  <img src="videos/demo.gif" alt="Delta Robot Demo" width="400">
</p>


## 📖 Overview

This repository contains the complete software and hardware interface for a 3-DOF Delta Robot built using **ROS 2 Humble**. The project provides robot visualization, kinematic calculations, hardware communication, and Arduino firmware for controlling the robot.

The robot is intended for high-speed pick-and-place tasks and serves as a modular platform for learning and experimentation with robotic manipulators.

### Features

- ROS 2 Humble packages
- URDF robot description
- RViz visualization
- Forward kinematics
- Inverse kinematics
- Arduino-based motor control
- Hardware interface package
- Workspace simulation

---
## 🔧 Hardware

The Delta Robot is driven by three stepper motors and uses:

- **3 Stepper Motors** for robot actuation.
- **Vacuum Pump** for object gripping.
- **IR Sensor** to detect whether an o
- bject has been successfully picked before continuing the task.

## 🤖 Arduino Firmware

The`arduino_codes` directory contains the firmware used to control the robot hardware.

### Files

- **pump_home_delta.ino**
  - Main firmware used during operation.
  - Controls the three stepper motors.
  - Executes the received trajectory.
  - Controls the vacuum pump.
  - Performs the homing procedure.


- **test_dir.ino**
  - Utility sketch used to verify the rotation direction of each stepper motor before running the complete system.
 
## ⚙️ Motion Control

The robot uses **ROS 2 Control** to generate motion trajectories.

The generated trajectory points are sent to the Arduino, which drives the three stepper motors while coordinating the vacuum pump during the pick-and-place task.
-
-**Arduino** to control the motors, pump, and sensors.
