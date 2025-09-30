

# METR4202 Team 15 – TurtleBot3 Exploration + Perception

This repository provides a minimal, robust ROS 2 (Humble) workspace for autonomous exploration (frontiers + Nav2) and ArUco perception on TurtleBot3 Waffle Pi. It avoids custom bringup/launch packages in favor of running our nodes directly while using official launch files for upstream stacks (SLAM Toolbox, Nav2, Gazebo, RViz2).

-----

## 1. Repository Structure

- `metr4202_2025_team15/team15_exploration`
  - `team15_exploration/explore_nav.py`: frontier exploration node. Consumes `/map`, selects frontiers, and sends goals to Nav2.
  - Hardcoded tuning constants are at the top of the file (no YAML needed for this node).

- `metr4202_2025_team15/team15_perception`
  - `team15_perception/aruco_detect_publish.py`: ArUco detection/smoothing node. Publishes `/targets` (PoseArray) and `/targets_viz` (MarkerArray).
  - Hardcoded tuning constants are at the top of the file (no YAML needed for this node).

- `metr4202_2025_team15/team15_exploration/config`
  - `slam_params.yaml`: SLAM Toolbox parameters (Prac 3 style).
  - `nav2_params.yaml`: Nav2 parameters (Prac 4 style DWB, costmaps, etc.).

- `metr4202_2025_team15/rviz`
  - Store your RViz2 configurations here (e.g., `my_config.rviz`).

- `metr4202_2025_team15/maps`
  - Save generated maps here (e.g., `office_map.pgm/.yaml`).

- `metr4202_2025_team15/worlds`
  - Put custom Gazebo worlds here (`.world`).

-----

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

-----

## 3. Recommended Launch Strategy (Prac 3–4 style)

We run our nodes directly with `ros2 run`. For SLAM and Nav2, we use their official launch files and pass our YAML configuration explicitly.

Use separate terminals for each component.

### A) Gazebo (Simulation)
- Built-in TurtleBot3 world (quick start):
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
- Custom world (place your file in `worlds/`):
```bash
ros2 launch gazebo_ros empty_world.launch.py \
  world:=/home/blaise/metr4202_ws/src/metr4202_2025_team15/worlds/<your_world>.world
```

### B) SLAM Toolbox (mapping mode)
```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/blaise/metr4202_ws/install/team15_exploration/share/team15_exploration/config/slam_params.yaml \
  use_sim_time:=true
```
Alternative (direct node):
```bash
ros2 run slam_toolbox online_async_node --ros-args \
  --params-file /home/blaise/metr4202_ws/install/team15_exploration/share/team15_exploration/config/slam_params.yaml
```

### C) Nav2 (navigation)
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/blaise/metr4202_ws/install/team15_exploration/share/team15_exploration/config/nav2_params.yaml \
  use_sim_time:=true
```

### D) RViz2 (optional)
- If you already saved a config into `rviz/`:
```bash
rviz2 -d /home/blaise/metr4202_ws/src/metr4202_2025_team15/rviz/my_config.rviz
```
- To create one: open `rviz2`, set displays (Fixed Frame: `map`, add Map, TF, LaserScan (`/scan`), MarkerArray (`/targets_viz`), etc.), then save via:
  File -> Save Config As… -> `metr4202_2025_team15/rviz/my_config.rviz`

### E) Our Nodes (run directly)
- Exploration node:
```bash
ros2 run team15_exploration explore_nav
```
- Perception node:
```bash
ros2 run team15_perception aruco_detect_publish
```

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
  params_file:=/home/blaise/metr4202_ws/install/team15_exploration/share/team15_exploration/config/nav2_params.yaml \
  use_sim_time:=true
```
- Full navigation with predefined map:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  map:=/home/blaise/metr4202_ws/src/metr4202_2025_team15/maps/<map_name>.yaml \
  params_file:=/home/blaise/metr4202_ws/install/team15_exploration/share/team15_exploration/config/nav2_params.yaml \
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


-----

## 9. Quick Reference

- Build: `colcon build --packages-select team15_exploration team15_perception`
- Source: `source install/setup.bash`
- Model: `export TURTLEBOT3_MODEL=waffle_pi`
- List topics: `ros2 topic list`
- Call service: `ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"`

-----

## 10. Notes  Pracs 3–4

- Prac 3 (SLAM): correct frames (`map`, `odom`, `base_link`/`base_footprint`), `scan_topic`, and resolution matter.
- Prac 4 (Nav2): tune controller critics, speeds, accels, inflation, and behavior tree.
- Start with the provided YAMLs and iterate during testing.