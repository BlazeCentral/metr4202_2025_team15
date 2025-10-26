METR4202 Team 15 – TurtleBot3 Exploration + Perception

This repository contains the software packages for autonomous exploration and ArUco marker perception on the TurtleBot3 Waffle Pi platform (ROS 2 Humble).

## 1. Overview of System

### 1.1. Repository Structure

* **team15_exploration:** Contains the `explore_nav.py` node for frontier selection and Nav2 goal setting.

* **team15_perception:** Contains the `aruco_detect_publish.py` node for ArUco detection and pose smoothing.

* **rviz:** RViz2 configuration files (e.g., `my_config.rviz`).

* **maps:** Generated map files (`.pgm` and `.yaml`).

* **worlds:** Custom Gazebo world files (`.world`).

### 1.2. Core Nodes and Topics

* **Node: explore_nav**

    * **Function:** Implements frontier-based exploration. Selects the next goal and submits it to Nav2.

    * **Subscribes:** `/map`, Nav2 Action Topics.

* **Node: aruco_detect_publish**

    * **Function:** Detects ArUco markers, smooths the estimated pose, and publishes results.

    * **Subscribes:** `/camera/image_raw`, `/tf`.

    * **Publishes:** `/targets` (PoseArray), `/targets_viz` (MarkerArray).

## 2. Install and Setup Instruction

### 2.0. One-time Setup and Building

#### Initial Installation (Cloning)

Navigate to your workspace source directory and clone the repository. Select the appropriate branch for your environment:

```bash
cd ~/metr4202_ws/src
# Use '-b hardware' for physical robot or '-b simulation' for Gazebo
git clone -b <branch_name> https://github.com/BlazeCentral/metr4202_2025_team15.git
```

Environment Setup
Run these commands in each new terminal session or add them to your ~/.bashrc. This sets the ROS environment and the robot model:

```bash

source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```
Build Workflow
After cloning, or whenever code changes are made, rebuild the packages:

```bash

cd ~/metr4202_ws
colcon build --packages-select team15_exploration team15_perception
```

2.1. Simulation Setup
Terminal 1 — Gazebo and SLAM/Navigation
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

System Nodes Launch Options
Launch any of the following options in separate terminals:

Aruco Only (Perception):

```bash


ros2 run team15_perception aruco_detect_publish
```


Exploration Only (Navigation):

```bash


ros2 run team15_exploration explore_nav
```

Full System (Both): Launch Aruco in one terminal and Exploration in another.

Bash

ros2 run team15_perception aruco_detect_publish
# In a new terminal:
ros2 run team15_exploration explore_nav
Terminal X — RViz2 Visualization (Simulation)
Bash

# Use a saved RViz config file:
rviz2 -d <path-to-repo>/rviz/<config_name>.rviz

# Or launch blank:
rviz2
RViz Configuration Check: Use ros2 topic list to verify all required topics (/map, /tf, /scan) are active. In the RViz interface, add displays for Map, RobotModel, LaserScan, /targets_viz (if Aruco is running), and any frontier visualization topics.

2.2. Hardware Setup
Pre-Launch Configuration (Workstation)
Set ROS Domain ID: Update your ~/.bashrc to match the robot's ID.

Bash


vim ~/.bashrc
# Add: export ROS_DOMAIN_ID={ROBOT_ID}
source ~/.bashrc
echo $ROS_DOMAIN_ID
Check Dependencies: Install required NumPy/SciPy versions for perception.

Bash

pip3 install scipy
python3 -m pip install --upgrade --force-reinstall "numpy==1.21.5" "scipy==1.8.0"
Terminal 1 — Robot Bringup (On TurtleBot via SSH)
Bash

ssh ubuntu@192.168.8.ID_NUMBER
ros2 launch turtlebot3_bringup robot.launch.py
Terminal 2 — Camera Node (On Workstation)
Bash

export TURTLEBOT3_MODEL=waffle_pi
ros2 run v4l2_camera v4l2_camera_node
Terminal 3 — Navigation/RViz (On Workstation)
Launches the real-time Nav2 stack and visualization. Note use_sim_time:=False for hardware.

Bash

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False slam:=True

# Use a saved RViz config file:
rviz2 -d <path-to-repo>/rviz/<config_name>.rviz
RViz Configuration Check: After launching, verify active topics (ros2 topic list). In RViz, confirm the map, robot, and visualization topics (especially /targets_viz for Aruco) are correctly added and displaying data.

Terminal 4/5 — System Nodes Launch Options (On Workstation)
Launch any of the following options in separate terminals:

Aruco Only (Perception):

Bash

ros2 run team15_perception aruco_detect_publish
Exploration Only (Navigation):

Bash

ros2 run team15_exploration explore_nav
Full System (Both): Launch Aruco in one terminal and Exploration in another.

Bash

ros2 run team15_perception aruco_detect_publish
# In a new terminal:
ros2 run team15_exploration explore_nav
3. References and Branch Description
3.1. Branch Description
This repository uses two primary branches:

simulation: Contains configurations, launch files, and parameters optimized for the Gazebo environment and simulated time (use_sim_time:=True).

hardware: Contains configurations, launch files, and parameters tailored for the physical TurtleBot3 robot, including real-time settings (use_sim_time:=False) and hardware driver dependencies.

3.2. Development Quick Reference
Build: colcon build --packages-select team15_exploration team15_perception

Source: source install/setup.bash

Set Model: export TURTLEBOT3_MODEL=waffle_pi

List Topics: ros2 topic list

Save Map: ros2 run nav2_map_server map_saver_cli -f <path>/<map_name>

Call Service (Example): ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"

3.3. Code Attribution
The following header provides attribution for the core node development:

/**
 ******************************************************************************
 * @file    explore_nav.py
 * @author  METR_TEAM_15
 * @date    6/10/2025
 * @brief   Interim Explore/Nav Node
 *
 * This node incorporates logic developed collaboratively by Team 15, drawing
 * inspiration from practical exercises and reference materials.
 ******************************************************************************
 */
