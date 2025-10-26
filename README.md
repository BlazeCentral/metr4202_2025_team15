

# METR4202 Team 15 – TurtleBot3 Exploration + Perception



-----

## 1. Repository Structure

- `metr4202_2025_team15/team15_exploration`
  - `team15_exploration/explore_nav.py`: frontier exploration node. Consumes `/map`, selects frontiers, and sends goals to Nav2.
  - Hardcoded tuning constants are at the top of the file (no YAML needed for this node).

- `metr4202_2025_team15/team15_perception`
  - `team15_perception/aruco_detect_publish.py`: ArUco detection/smoothing node. Publishes `/targets` (PoseArray) and `/targets_viz` (MarkerArray).
  - Hardcoded tuning constants are at the top of the file (no YAML needed for this node).


- `metr4202_2025_team15/rviz`
  - Store your RViz2 configurations here (e.g., `my_config.rviz`).

- `metr4202_2025_team15/maps`
  - Save generated maps here (e.g., `office_map.pgm/.yaml`).

- `metr4202_2025_team15/worlds`
  - Put custom Gazebo worlds here (`.world`).

-----

# SIMULATION SETUP

## 2. One-time Setup (per terminal)

Add these lines to your `~/.bashrc` (recommended), or run in each new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/metr4202_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

Build after pulls/changes:
```bash
cd ~/metr4202_ws
colcon build --packages-select team15_exploration team15_perception
```


## 4.  Combined SLAM+Nav2 (3 Terminal Workflow)

If you prefer fewer terminals, you can combine SLAM and Nav2:

### Terminal 1 — Gazebo (headless)
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 — TurtleBot3 Navigation2 (Nav2 + SLAM combined)
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  slam:=True use_sim_time:=True \
  
```

### Terminal 3 — Our Exploration node
```bash
ros2 run team15_exploration explore_nav
```


### Terminal 4 — Our Aruco node
```bash
ros2 run team15_perception aruco_detect_publish
```

-----
-----

## 4. Running Modes

- Perception only: Gazebo (or real robot) + RViz + `aruco_detect_publish`.
- Exploration only: Gazebo + SLAM + Nav2 + `explore_nav`.
- Full system: Gazebo + SLAM + Nav2 + RViz + both nodes.

Tip: Always verify `/map`, `/tf`, `/scan`, Nav2 bringup status, and topics with `ros2 topic list`.

-----

## 5. RViz2 Configuration (save and reuse)

- Start RViz2, set displays and frames, then save the config into `rviz/`:
  - File -> Save Config As… -> `metr4202_2025_team15/rviz/my_config.rviz`
- Use it next time:
```bash
rviz2 -d /home/blaise/metr4202_ws/src/metr4202_2025_team15/rviz/my_config.rviz
```

-----

## 6. Maps: Save, Load, and Use

### Save a map (while SLAM is running)
```bash
ros2 run nav2_map_server map_saver_cli -f /home/blaise/metr4202_ws/src/metr4202_2025_team15/maps/<map_name>
```
This creates `<map_name>.pgm` and `<map_name>.yaml` in `maps/`.

### Use a saved map (localization mode)
- Localization-only:
```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/blaise/metr4202_ws/src/metr4202_2025_team15/maps/<map_name>.yaml \
  use_sim_time:=true
```
- Full navigation with predefined map:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  map:=/home/blaise/metr4202_ws/src/metr4202_2025_team15/maps/<map_name>.yaml \
  use_sim_time:=true
```

-----

## 7. Gazebo Worlds

### Use TurtleBot3 default worlds
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Use custom worlds
1) Place `.world` files into `worlds/`.
2) Launch Gazebo with your world:
```bash
ros2 launch gazebo_ros empty_world.launch.py \
  world:=/home/blaise/metr4202_ws/src/metr4202_2025_team15/worlds/<your_world>.world
```

# HARDWARE SETUP

# TERM1 - ON TURTLEBOT

 ssh ubuntu@192.168.8.ID_NUMBER
  ros2 launch turtlebot3_bringup robot.launch.py
# TERM2 - Camera 
Export TURTLEBOT: export TURTLEBOT3_MODEL=waffle_pi
ROS2 Run Camera node: Ros2 run v4l2_camera v4l2_camera_node (l not 1)


# TERM3 - NAV2/RVIS
CHANGE BASH FILE
Open Bash: vim ~/.bashrc

Change the ROSID in bashrc: ROS Domain ID = {ROBOT_ID}

Source Bashrc (in the ~): source ~/.bashrc 

Echo the number to check its correct: echo $ROS_DOMAIN_ID


Start up robot nav2: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False slam:=True

cHECK topics in new terminal
Check topics:ros2 topic list (in new terminal)


After a wait, add in the targets/targets_viz topics onto the rViz 
ALSO ADD THE FRONTIER TOPIC?

 # TERM4 - ARUCO DETECTIOn NODE
 


IF NOT ALREADY INSTALLED/ ON CORRECT VERSION 
Check NUMPY: 1.21.5 and 1.8.0:
pip3 install scipy
Python3 -m pip install –upgrade –force-reinstall “numpy==1.21.5” “scipy==1.8.0”

THEN GIT CLONE AND BUILD
cd metr4202_ws/src
export TURTLEBOT3_MODEL=waffle_pi
source 
git clone https://github.com/BlazeCentral/metr4202_2025_team15.git
source install/bash
colcon build


LAUNCH IT 
ros2 run team15_perception aruco_detect_publish


 # TERM5- EXPLORATION NODE

cd metr4202_ws/src
export TURTLEBOT3_MODEL=waffle_pi
ros2 run team15_exploration explore_nav

-----

## 9. Quick Reference

- Build: `colcon build --packages-select team15_exploration team15_perception`
- Source: `source install/setup.bash`
- Model: `export TURTLEBOT3_MODEL=waffle_pi`
- List topics: `ros2 topic list`
- Call service: `ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"`

-----

## 10. Referencing
/**
 ******************************************************************************
 * @file   explore_nav.py
 * @author  METR_TEAM_15
 * @date    6/10/2025
 * @brief   Interim Explore/Nav Node
 *
 * REF: Gemini AI was conversed with the develop a pseudocode with inspiration from pracs and provided files.
    Our team worked together to get a proper flow diagram sufficient enough to the point
   Where we passed it into Gemini and turned it into working python code which was implemented
   in the explore_nav node.
   REF: Other Areas AI was used was in developing ideas on fixing issues such as blacklist,
   generating the file structure diagram, fixing launch issues and generating readme and refining comments.
 ******************************************************************************
 */
