import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup_share = get_package_share_directory('team15_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_params_file = os.path.join(pkg_bringup_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_bringup_share, 'config', 'slam_params.yaml')
    aruco_params_file = os.path.join(pkg_bringup_share, 'config', 'aruco_params.yaml')

    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Start SLAM Toolbox
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_params_file}.items(),
    )

    # Start the full Nav2 system with our parameters
    # This includes the controller, planner, behavior tree, and RViz
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'map': '""', 'params_file': nav2_params_file}.items(),
    )

    # Launch the exploration node
    start_explore_nav_node_cmd = Node(
        package='team15_exploration',
        executable='explore_nav',
        name='explore_nav',
        output='screen',
    )

    # Launch the perception node
    start_aruco_detect_node_cmd = Node(
        package='team15_perception',
        executable='aruco_detect_publish',
        name='aruco_detect_publish',
        output='screen',
        parameters=[aruco_params_file],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        start_gazebo_cmd,
        start_slam_toolbox_cmd,
        start_nav2_cmd,
        start_explore_nav_node_cmd,
        start_aruco_detect_node_cmd,
    ])