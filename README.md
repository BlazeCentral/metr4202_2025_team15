# METR4202 Team Project – Autonomous Exploration & Target Search

**Team Members:** Blaise, Harveer, Genevieve, Aiden <br>
**Semester:** 2, 2025

---

## 1. Project Goal

The objective of this project is to develop a fully autonomous system for a TurtleBot3 Waffle Pi. The system must be capable of operating in an unknown environment to perform the following key tasks:

-   **Explore & Map:** Autonomously navigate the environment to build a 2D map using SLAM.
-   **Detect & Localize:** Identify ArUco markers and report their positions within the map with ≤0.5m accuracy.
-   **Integrate & Navigate:** Utilize the ROS2 Nav2 stack for all path planning and collision avoidance.

The final deliverable must function in both **Gazebo simulation** and on the **physical hardware**.

---

## 2. Quick Start: How to Run the Simulation

This is the primary method for testing the complete, integrated system.

1.  **Open a Terminal** and navigate to the ROS2 workspace root.
    ```bash
    cd ~/metr4202_ws
    ```

2.  **Build the Package:** Before launching, always build the package to apply any code changes. Use `--packages-select` for a faster build.
    ```bash
    colcon build --packages-select metr4202_2025_team15
    ```

3.  **Source the Workspace:**
    ```bash
    source install/setup.bash
    ```

4.  **Run the Main Launch File:** This single command starts Gazebo, SLAM, Nav2, RViz, and all custom project nodes.
    ```bash
    ros2 launch metr4202_2025_team15 sim_explore.launch.py
    ```

5.  **Initiate Exploration:** Once all windows are loaded, open a **new terminal**, source the workspace (`source install/setup.bash`), and call the service to start the autonomous exploration loop.
    ```bash
    ros2 service call /get_next_waypoint std_srvs/srv/Trigger "{}"
    ```
    The robot will now begin exploring the environment on its own.

---

## 3. File System Breakdown

This table describes the purpose of every file within our `metr4202_2025_team15` package.

| Path & File | Description |
| :--- | :--- |
| **`package.xml`** | **Package Passport.** <br> Defines the package name, version, author, and most importantly, its **dependencies** (e.g., `rclpy`, `nav_msgs`). `colcon` reads this file first to build the project correctly. |
| **`setup.py`** | **Assembly Instructions.** <br> Tells `colcon` how to install the package. It specifies how to install `launch` and `config` files and defines the `entry_points` that make our Python scripts runnable with `ros2 run`. |
| **`launch/sim_explore.launch.py`** | **The Director.** <br> This is our main launch file. It automates the entire simulation startup process by launching Gazebo, SLAM Toolbox, Nav2, and our custom nodes, passing the correct parameters to each. Use this for all integrated testing. |
| **`nodes/explore_nav.py`** | **The Brain.** <br> This node is responsible for the "think/act" part of our system. It analyzes the map from SLAM, finds unexplored frontiers, and sends navigation goals to Nav2. |
| **`nodes/aruco_detect_publish.py`** | **The Eyes.** <br> This node handles the "see" part of our system. It processes the camera feed, detects ArUco markers, calculates their 3D pose, and publishes them to the `/targets` topic. |
| **`config/slam_params.yaml`** | **SLAM Tuner.** <br> Contains configuration parameters for the SLAM Toolbox, such as map resolution and frame IDs. Allows us to fine-tune mapping performance without changing code. |
| **`config/nav2_params.yaml`** | **Navigation Tuner.** <br> Contains parameters for the Nav2 stack. This file is used to configure path planners, controllers, and recovery behaviors. |
| **`config/aruco_params.yaml`** | **ArUco Tuner.** <br> Parameters for our `aruco_detect_publish` node, such as marker size and dictionary type. |

---

## 4. Notes for the Team

-   **Always Build:** After you `git pull` or make any changes to Python files, launch files, or `setup.py`, you **must** run `colcon build` for your changes to take effect.
-   **Always Source:** In every new terminal you open, you must source the workspace with `source install/setup.bash` before you can use `ros2` commands with our package. Consider adding this to your `~/.bashrc` file.
-   **Use the Launch File:** Do not run nodes individually for integrated testing. The `sim_explore.launch.py` file is configured to ensure all nodes start with the correct parameters (like `use_sim_time:=True`), which is crucial for the simulation to work correctly.
-   **Check `colcon` Output:** Pay close attention to the output of `colcon build`. It will tell you about syntax errors or missing dependencies before you even try to run the code.

---

## 5. Git Workflow & Common Commands

A consistent Git workflow is essential for team collaboration. Please create a new branch for each feature or bugfix.

| Action | Command | Notes |
| :--- | :--- | :--- |
| **Clone Repository** | `git clone <repo_url>` | Do this only once for the initial setup. |
| **Check Status** | `git status` | Shows your current branch and any modified files. |
| **Create New Branch** | `git checkout -b <branch_name>` | Use a descriptive name, e.g., `feature/aruco-smoothing`. |
| **Switch Branch** | `git checkout <branch_name>` | Move to an existing branch. |
| **Get Latest Changes** | `git pull origin main` | **Always do this before starting new work** to avoid conflicts. |
| **Stage Changes** | `git add .` | Stages all modified files for committing. |
| **Commit Changes** | `git commit -m "Your descriptive message"` | Write a clear message explaining *what* and *why*. |
| **Push to GitHub** | `git push origin <branch_name>` | Upload your committed changes to your branch. |
| **Merge into Main** | **Open a Pull Request** on GitHub. | This is the preferred method. It allows for code review before merging into the `main` branch, preventing errors. |