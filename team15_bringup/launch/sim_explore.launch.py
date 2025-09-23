import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup_share = get_package_share_directory('team15_bringup')
    
    # Declare launch arguments for custom nodes
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    aruco_params_file = os.path.join(pkg_bringup_share, 'config', 'aruco_params.yaml')

    # Launch the exploration node from its own package
    start_explore_nav_node_cmd = Node(
        package='team15_exploration',
        executable='explore_nav',
        name='explore_nav',
        output='screen'
    )

    # Launch the perception node from its own package
    start_aruco_detect_node_cmd = Node(
        package='team15_perception',
        executable='aruco_detect_publish',
        name='aruco_detect_publish',
        output='screen',
        parameters=[aruco_params_file]
    )
    
    # Define a launch description with just your custom nodes
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        start_explore_nav_node_cmd,
        start_aruco_detect_node_cmd,
    ])