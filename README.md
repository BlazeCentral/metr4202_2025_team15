

# METR4202 Team Project – Autonomous Exploration & Target Search

This repository contains the codebase for the **METR4202 Robotics & Automation** team project. It is structured as a professional multi-package ROS2 workspace.

-----

## 1\. Project Information & Deadlines

| Item | Details |
| :--- | :--- |
| **Team Members** | Blaise, Harveer, Genevieve, Aiden |
| **Goal** | Develop an autonomous exploration and ArUco marker detection system for the TurtleBot3 Waffle Pi. |
| **Interim Demo Due** | `Friday, Week 9 (e.g., 26th September 2025)` |
| **Final Demo Due** | `Friday, Week 13 (e.g., 24th October 2025)` |

-----

## 2\. Full System Simulation: From Zero to Autonomous

This is the primary workflow. Follow these steps precisely from a **fresh terminal** to get the entire system running in Gazebo.

#### **Step 1: Navigate to the Workspace**

All commands must be run from the root of your ROS2 workspace.

```bash
cd ~/metr4202_ws
```

#### **Step 2: Build All Project Packages**

This command finds and builds all three of our custom packages (`team15_exploration`, `team15_perception`, `team15_bringup`). Always run this after pulling new changes or editing code.

```bash
colcon build --packages-up-to team15_bringup
```

**✅ Expect:** `Summary: 3 packages finished`.

#### **Step 3: Source the Workspace**

This crucial step makes your newly built packages, nodes, and launch files available to ROS2 in your current terminal.

```bash
source install/setup.bash
```

#### **Step 4: Set the Robot Model**

You must export the `TURTLEBOT3_MODEL` environment variable for Gazebo to load the correct robot.

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

#### **Step 5: Launch the System**

This single command starts everything: Gazebo, SLAM, Nav2, RViz, and our custom nodes.

```bash
ros2 launch team15_bringup sim_explore.launch.py
```

#### **Step 6: Initiate Exploration**

The robot will wait for your command. Open a **new terminal**, source the workspace again, and call the service to begin autonomous exploration.

```bash
# In a NEW terminal
cd ~/metr4202_ws
source install/setup.bash
ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"
```

The robot will now begin exploring the environment on its own. To send it to the next frontier, simply run the `ros2 service call` command again after it completes its current goal.

-----

## 3\. Package Structure Explained

Our project is organized into three distinct ROS2 packages located inside the `metr4202_2025_team15` directory. This separation of concerns makes the system cleaner and easier to manage.

| Package | Location | Purpose | Key Files |
| :--- | :--- | :--- | :--- |
| **`team15_exploration`**| `metr4202_2025_team15/team15_exploration` | **The Brain 🧠**<br>Contains the logic for autonomous exploration. | `nodes/explore_nav.py` |
| **`team15_perception`** | `metr4202_2025_team15/team15_perception` | **The Eyes 👀**<br>Contains the logic for detecting ArUco markers. | `nodes/aruco_detect_publish.py` |
| **`team15_bringup`** | `metr4202_2025_team15/team15_bringup` | **The Director 🎬**<br>Contains no nodes. Its only job is to launch and configure all other parts of the system. | `launch/sim_explore.launch.py`<br>`config/*.yaml`<br>`README.md` |

-----

## 4\. How to Test Individual Components

For debugging, you can run and test each node in isolation.

#### **Testing the Perception Node (`aruco_detect_publish`)**

1.  **Terminal 1: Launch Gazebo & SLAM**
    ```bash
    # (Build and source your workspace first as per steps 2.1-2.3)
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch slam_toolbox online_async_launch.py
    ```
2.  **Terminal 2: Run the Perception Node with Parameters**
    ```bash
    ros2 run team15_perception aruco_detect_publish --ros-args --params-file install/team15_bringup/share/team15_bringup/config/aruco_params.yaml
    ```
3.  **Terminal 3: Run RViz & Keyboard Control**
    ```bash
    rviz2 & # The '&' runs it in the background
    ros2 run turtlebot3_teleop teleop_keyboard
    ```
4.  **Verification:** Drive the robot to face an ArUco marker. In RViz, add the `/targets_viz` topic (Type: `MarkerArray`) and verify that a visualization appears. Echo the `/targets` topic to see the `PoseArray` output.

#### **Testing the Exploration Node (`explore_nav`)**

1.  **Terminal 1: Launch Gazebo & SLAM**
    ```bash
    # (Build and source your workspace first)
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch slam_toolbox online_async_launch.py
    ```
2.  **Terminal 2: Run the Exploration Node**
    ```bash
    ros2 run team15_exploration explore_nav
    ```
3.  **Terminal 3: Manually Build a Partial Map**
    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```
    Drive the robot around for \~30 seconds to create clear frontiers on the map.
4.  **Verification:** Call the `/get_next_waypoint` service. The node should log that it is sending a goal. You will see errors because Nav2 isn't running, but this confirms the frontier logic is working.

-----

## 5\. Key Things to Remember

  - **Always Build:** After a `git pull` or any code change, run `colcon build` for the changes to take effect.
  - **Always Source:** In **every new terminal**, you must run `source install/setup.bash` to make our packages available.
  - **Always Export Model:** You must run `export TURTLEBOT3_MODEL=waffle_pi` before any launch command involving the robot.
  - **Add all three source commands to your `~/.bashrc` file for convenience\!**
    ```bash
    # Add these lines to the end of ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/metr4202_ws/install/setup.bash" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
    ```

-----

## 6\. Commonly Used Commands

| Action | Command |
| :--- | :--- |
| **See All Active Topics** | `ros2 topic list` |
| **Check a Topic's Output** | `ros2 topic echo /scan` |
| **See Active Nodes** | `ros2 node list` |
| **Call Exploration Service**| `ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"` |
| **Send a Manual Nav Goal** | `ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{...}"` |

-----

## 7\. Git Workflow

| Action | Command | Notes |
| :--- | :--- | :--- |
| **Clone Repository** | `git clone <repo_url>` | Do this only once for the initial setup. |
| **Check Status** | `git status` | Shows your current branch and any modified files. |
| **Create New Branch** | `git checkout -b <branch_name>` | Use a descriptive name, e.g., `feature/aruco-smoothing`. |
| **Get Latest Changes**| `git pull origin main` | **Always do this before starting new work** to avoid conflicts. |
| **Stage & Commit** | `git add .`<br>`git commit -m "Your descriptive message"` | Write a clear message explaining *what* and *why*. |
| **Push to GitHub** | `git push origin <branch_name>` | Upload your committed changes to your branch. |
| **Merge into Main** | **Open a Pull Request** on GitHub. | This is the preferred method for code review. |