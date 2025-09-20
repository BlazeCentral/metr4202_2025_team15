Of course. Here is the concise, command-focused testing procedure suitable for a `README.md` file. It's written in GitHub-Flavored Markdown and includes multiple distinct tests with clear initialization steps based on your practicals.

You can copy and paste the text below directly into your `README.md` file.

```markdown
# METR4202 Project Testing Procedure

This document outlines the steps to verify the functionality of the autonomous exploration system in Gazebo.

---

### **Test Suite 1: Environment & Build Verification**

**Goal:** Ensure the workspace is correctly built and the base simulation launches without errors.

| Step | Action | Command | Expected Result |
| :--- | :--- | :--- | :--- |
| **1.1** | **Navigate & Build** | `cd ~/metr4202_ws`<br>`colcon build --packages-select metr4202_2025_team15` | The build completes with `0 errors`. |
| **1.2** | **Source Workspace** | `source install/setup.bash` | No errors are shown. |
| **1.3** | **Launch Baseline Sim** | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` | Gazebo window opens with the TurtleBot3 in the default world. |

*After verifying, close the simulation (`Ctrl+C`).*

---

### **Test Suite 2: ArUco Marker Detection (`aruco_detect_publish`)**

**Goal:** Verify that the perception node can correctly identify ArUco markers and publish their pose in the `map` frame.

| Step | Action | Command | Expected Result |
| :--- | :--- | :--- | :--- |
| **2.1** | **(T1) Launch Sim & SLAM** | `ros2 launch slam_toolbox online_async_launch.py` | Gazebo starts, and SLAM begins building a map. |
| **2.2** | **(T2) Launch ArUco Node** | `ros2 run metr4202_2025_team15 aruco_detect_publish` | Node starts and waits for camera data. |
| **2.3** | **(T3) Launch RViz** | `rviz2` | RViz opens. <br> 1. Set Fixed Frame to `map`. <br> 2. Add `/map`, `/scan`, TF, `/camera/image_raw`, and `/targets_viz` topics. |
| **2.4** | **(T4) Start Manual Control** | `ros2 run turtlebot3_teleop teleop_keyboard` | Teleop node starts. |
| **2.5** | **Verify Detection** | Drive the robot until a marker is in view of the camera. | **In RViz:**<br> • A visualization marker appears over the ArUco marker's location.<br> **In a new terminal:**<br> `ros2 topic echo /targets`<br> • `PoseArray` messages are published with `frame_id: 'map'`. |



---

### **Test Suite 3: Frontier Exploration (`explore_nav`)**

**Goal:** Verify that the exploration node can identify frontiers and dispatch a valid goal to Nav2.

| Step | Action | Command | Expected Result |
| :--- | :--- | :--- | :--- |
| **3.1** | **(T1) Launch Sim & SLAM** | `ros2 launch slam_toolbox online_async_launch.py` | Gazebo and SLAM are running. |
| **3.2** | **(T2) Launch Manual Control** | `ros2 run turtlebot3_teleop teleop_keyboard` | Drive the robot for ~20 seconds to create a partial map with clear frontiers. |
| **3.3** | **(T3) Launch Exploration Node** | `ros2 run metr4202_2025_team15 explore_nav` | Node starts and is ready for service calls. |
| **3.4** | **Verify Frontier Goal** | In a new terminal (`source install/setup.bash`):<br>`ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"` | • Service returns `success: True`.<br> • The robot begins navigating autonomously towards the edge of the known map. |

---

### **Test Suite 4: Full System Integration**

**Goal:** Verify the complete autonomous loop: the robot explores, maps, and detects markers without any manual intervention after the initial trigger.

| Step | Action | Command | Expected Result |
| :--- | :--- | :--- | :--- |
| **4.1** | **(T1) Launch Everything** | `ros2 launch metr4202_2025_team15 sim_explore.launch.py` | A single command launches Gazebo, SLAM, Nav2, RViz, and both custom nodes. |
| **4.2** | **Kick-off Exploration** | In a new terminal (`source install/setup.bash`):<br>`ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"` | The robot begins moving autonomously. |
| **4.3** | **Verify Loop** | Let the system run. Drive the robot past a marker if needed. | • The map in RViz grows as the robot explores.<br> • Nav2 successfully avoids obstacles.<br> • When the robot reaches its goal, it stops.<br> • Calling the service again sends it to a *new* frontier. |

---

### **Test Suite 5: Robustness Testing**

**Goal:** Ensure the system handles common failure scenarios gracefully.

| Test | Action | Expected Result |
| :--- | :--- | :--- |
| **Unreachable Goal** | While the robot is navigating, use the Gazebo UI to place a wall in its path. | Nav2 will fail the goal. Your `explore_nav` node should log the failure, blacklist the goal, and be ready to select a *different* frontier on the next service call. |
| **Kidnapped Robot** | Use the Gazebo UI to move the robot to a different, already-mapped location. | The navigation goal will fail. SLAM's particle filter should reconverge at the new correct location. The system should recover and be ready to explore from the new position. |
| **No Frontiers Left** | Allow the robot to fully map the environment. Then, call the exploration service again. | The service should return `success: False` with a message like "No valid frontiers found." The robot should remain stationary, and the node should not crash. |
```