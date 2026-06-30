
# Search & Rescue Robot — ROS2 + Gazebo + SLAM

Autonomous search and rescue simulation using TurtleBot3, demonstrating LIDAR-based SLAM mapping, color-based victim detection, and a finite state machine mission controller — built and tested on The Construct Sim (ROS2 Humble).

## Overview

This project simulates a robot navigating an unknown environment to map terrain and locate "victims" (represented as colored markers), generating a final map and mission report — a common pattern in real-world search and rescue robotics.

## Features

- SLAM-based mapping: Real-time occupancy grid mapping using slam_toolbox / Cartographer with LIDAR data, visualized live in RViz2.
- Color-based victim detection: A vision node (victim_detector) processes the robot's camera feed in HSV color space to detect red markers simulating victims, publishing detection events with bearing/position data.
- FSM mission controller: A finite state machine (mission_controller) drives autonomous exploration with simple LIDAR-based obstacle avoidance, transitioning between EXPLORE, INVESTIGATE, and REPORT states whenever a victim is detected.
- Map and mission reporting: Automated script saves the final occupancy grid map and generates a text-based mission report summarizing the search.

## Tech Stack

- ROS2 Humble
- Gazebo (TurtleBot3 Burger model)
- slam_toolbox (online async SLAM)
- OpenCV and cv_bridge for vision processing
- Python 3 (rclpy)

## Package Structure

search_rescue_robot/
- search_rescue_robot/
  - __init__.py
  - victim_detector.py (color-based victim detection node)
  - mission_controller.py (FSM-based exploration and reporting node)
- scripts/
  - generate_report.py (saves map and writes mission report)
- package.xml
- setup.py
- README.md

## Setup and Build

cd ~/ros2_ws/src
git clone YOUR_REPO_URL_HERE search_rescue_robot
cd ~/ros2_ws
colcon build --packages-select search_rescue_robot
source install/setup.bash

## Running the Simulation

Each command runs in its own sourced terminal: source /opt/ros/humble/setup.bash followed by export TURTLEBOT3_MODEL=burger

1. Launch Gazebo world:
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2. Launch SLAM:
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
(or ros2 launch slam_toolbox online_async_launch.py)

3. Visualize in RViz2:
rviz2
Add Map (topic /map) and LaserScan (topic /scan) displays, set Fixed Frame to map.

4. Run victim detector:
ros2 run search_rescue_robot victim_detector

5. Run mission controller (autonomous exploration) or drive manually with teleop. Do not run both at once, they both publish to /cmd_vel:
ros2 run search_rescue_robot mission_controller
or
ros2 run turtlebot3_teleop teleop_keyboard

6. Generate final map and report:
python3 scripts/generate_report.py
Outputs a .pgm/.yaml map and a .txt mission report to ~/sar_report/


