# METR4202 Team 15 – TurtleBot3 Exploration + Perception

This repository contains the software packages for autonomous exploration and ArUco marker perception on the TurtleBot3 Waffle Pi platform (ROS 2 Humble).

# 1. Overview of System

## 1.1. Repository Structure

#### team15_exploration:
Contains the `explore_nav.py` node for frontier selection and Nav2 goal setting.

#### team15_perception:
Contains the `aruco_detect_publish.py` node for ArUco detection and pose smoothing.

<img width="1156" height="501" alt="METR4202IO drawio" src="https://github.com/user-attachments/assets/ab3e5afc-3aa5-4382-a3cb-5f96c92a8cdd" />

  
## 1.2 Branch Structure
This repository uses two primary branches:

#### simulation:
Contains configurations, launch files, and parameters optimized for the Gazebo environment and simulated time (use_sim_time:=True).

#### hardware: 
Contains configurations, launch files, and parameters tailored for the physical TurtleBot3 robot, including real-time settings (use_sim_time:=False) and hardware driver dependencies.


# 2. Install and Setup Instruction

## 2.0. One-time Setup and Building

#### Initial Installation (Cloning)

Navigate to your workspace source directory and clone the repository. Select the appropriate branch for your environment:

```bash
cd ~/metr4202_ws/src
# Use '-b hardware' for physical robot or '-b simulation' for Gazebo
git clone -b <branch_name> https://github.com/BlazeCentral/metr4202_2025_team15.git
```

#### Environment Setup
Run these commands in each new terminal session or add them to your ~/.bashrc. This sets the ROS environment and the robot model:

```bash

source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```
#### Build Workflow
After cloning, or whenever code changes are made, rebuild the packages:

```bash

cd ~/metr4202_ws
colcon build
```
---
## 2.1. Simulation Setup
#### Terminal 1 — Gazebo and SLAM/Navigation
Launches the simulated robot and the combined SLAM and Nav2 stack.

```bash

# Export robot model
export TURTLEBOT3_MODEL=waffle_pi

# Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Launch Nav2 + SLAM
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
    slam:=True use_sim_time:=True
```

#### System Nodes Launch Options
Launch any of the following options in separate terminals:

##### Aruco Only (Perception):

```bash


ros2 run team15_perception aruco_detect_publish
```


##### Exploration Only (Navigation):

```bash


ros2 run team15_exploration explore_nav
```

###### Full System (Both): Launch Aruco in one terminal and Exploration in another.

```bash

ros2 run team15_perception aruco_detect_publish
# In a new terminal:
ros2 run team15_exploration explore_nav
```


## 2.2. Hardware Setup

### Pre-Launch Configuration (Workstation)
Set ROS Domain ID: Update your ~/.bashrc to match the robot's ID.

```bash



vim ~/.bashrc
# Add: export ROS_DOMAIN_ID={ROBOT_ID}
source ~/.bashrc
echo $ROS_DOMAIN_ID
```

### Check Dependencies: Install required NumPy/SciPy versions for perception.

```bash
pip3 install scipy
python3 -m pip install --upgrade --force-reinstall "numpy==1.21.5" "scipy==1.8.0"
```
(You may still get an error with one of the function names in explore nav - terminal should tell you what to change it to though)

### Terminal 1 — Robot Bringup (On TurtleBot via SSH)
```bash
ssh ubuntu@192.168.8.ID_NUMBER
ros2 launch turtlebot3_bringup robot.launch.py
```

### Terminal 2 — Camera Node (On Workstation)
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run v4l2_camera v4l2_camera_node
```
### Terminal 3 — Navigation/RViz (On Workstation)
Launches the real-time Nav2 stack and visualization. Note use_sim_time:=False for hardware.

```bash


ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False slam:=True
```

#### RViz Configuration Check:
After launching, verify active topics (ros2 topic list). In RViz, confirm the map, robot, and visualization topics (especially /targets_viz for Aruco) are correctly added and displaying data.

### Terminal 4/5 — System Nodes Launch Options (On Workstation)
Launch any of the following options in separate terminals:

#### Aruco Only (Perception):

```bash


ros2 run team15_perception aruco_detect_publish
```
#### Exploration Only (Navigation):

```bash


ros2 run team15_exploration explore_nav
```
#### Full System (Both): Launch Aruco in one terminal and Exploration in another.
First launch:

```bash

ros2 run team15_perception aruco_detect_publish
```

Then in another terminaL:

```bash

ros2 run team15_exploration explore_nav
```

# 3. References and Branch Description

# Online Reference:
The following Github repository was used to help with the development of the Aruco Detection Node:
https://github.com/Rishit-katiyar/ArUcoMarkerDetector/tree/main

The repository of provided aruco course resources was also used to help with development: https://github.com/METR4202/metr4202_aruco_explore
## AI:
A combination of Cursor AI, ChatGPT and Gemini was used to develop much of the foundations of the code. Pseudocode and flow charts where inserted into the chats in order to intelligently develop out ideas into working prototype code which were adjusted and customised for the robot based on our needs and performance. The code was finalised with human hands especially the weightings and specific parameters, and the heuristic scoring code.

# Documents
Pracs 1-5 and Assessment Documentation
