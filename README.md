# Map

This repository contains the implementation of a **mapping system** that uses a combination of sensors and algorithms to track and update a robot's movement and surrounding environment. The primary goal is to build an accurate map using available data like odometry and sensor readings.

## Features
- **Real-time mapping:** Efficiently updates the map based on sensor data.
- **Robot Pose Estimation:** Uses sensor fusion (IMU and encoder data) to track the robot's pose.
- **Supports ROS:** Built on ROS for better robotics control and integration.
- **Extended Kalman Filter (EKF) Implementation:** Implements the EKF to smooth and predict robot movements.

## Technologies Used
- **ROS Noetic**
- **Python**
- **IMU sensor** (MPU 9250)
- **Odometry data** (from encoders)

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/ahmedanwar123/map.git
   ```
2. Navigate into the project folder:
   ```bash
   cd map
   ```
3. Build the project:
   If you are using ROS with a catkin workspace:
   ```bash
   catkin_make
   source devel/setup.bash
   ```
## Usage

Launch the mapping node:
   ```bash
   roslaunch map map.py
   ```
