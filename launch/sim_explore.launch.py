import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    This is the main launch file for the simulation.
    It launches Gazebo, SLAM Toolbox, Nav2, and the custom project nodes.
    This follows the patterns from Prac 3 and 4 for launching integrated systems.
    """
    # Get the share directory for the project package
    pkg_share = get_package_share_directory('metr4202_2025_team15')
    
    # Get the share directory for turtlebot3_gazebo
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # --- Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # --- Configuration File Paths ---
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    # aruco_params_file = os.path.join(pkg_share, 'config', 'aruco_params.yaml') # For when aruco_detect is ready

    # --- Gazebo Simulation ---
    # This launches the TurtleBot3 in a specified world.
    # We are using the turtlebot3_world as a default for now.
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # --- SLAM Toolbox ---
    # This runs SLAM to build the map, as shown in Prac 3.
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items(),
    )

    # --- Navigation (Nav2) Stack ---
    # This launches the full Nav2 stack, including planners, controllers, and AMCL (though SLAM provides localization here).
    # This is based on the Prac 4 launch command.
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )

    # --- Custom Project Nodes ---
    # Starts the autonomous exploration node.
    start_explore_nav_node_cmd = Node(
        package='metr4202_2025_team15',
        executable='explore_nav', # This must match the entry_point in setup.py
        name='explore_nav_node',
        output='screen'
    )

    # Starts the ArUco detection node (commented out until the file is created).
    # start_aruco_detect_node_cmd = Node(
    #     package='metr4202_2025_team15',
    #     executable='aruco_detect_publish', # This must match the entry_point in setup.py
    #     name='aruco_detect_node',
    #     output='screen',
    #     parameters=[aruco_params_file]
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        
        # Start Gazebo
        start_gazebo_cmd,
        
        # Start SLAM and Nav2
        start_slam_toolbox_cmd,
        start_nav2_cmd,

        # Start custom nodes
        start_explore_nav_node_cmd,
        # start_aruco_detect_node_cmd, # Uncomment when ready
    ])