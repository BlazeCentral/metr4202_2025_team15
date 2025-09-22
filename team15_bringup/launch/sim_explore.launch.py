import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # This launch file now correctly finds resources from the 'team15_bringup' package
    pkg_bringup_share = get_package_share_directory('team15_bringup')
    
    # Yaml files
    nav2_params_file = os.path.join(pkg_bringup_share, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_bringup_share, 'config', 'slam_params.yaml')
    aruco_params_file = os.path.join(pkg_bringup_share, 'config', 'aruco_params.yaml')
    
    # RVIS File (added)
    rviz_config_file = os.path.join(pkg_bringup_share, 'rviz', 'main.rviz') # New: path to rviz config

    #Simulation time clock
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # start gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Start slam
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_params_file}.items(),
    )

    # Launch NAV2
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params_file}.items(),
    )

    # Launch the exploration node from its own package
    start_explore_nav_node_cmd = Node(
        package='team15_exploration',
        executable='explore_nav',
        name='explore_nav',
        output='screen',
        # ADDED: Wait for the slam_toolbox node to become available
        on_startup=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
                ),
                launch_arguments={'slam_params_file': slam_params_file, 'wait_for_slam': 'true'}.items(),
            )
        ]
    )


    # Launch the perception node from its own package
    start_aruco_detect_node_cmd = Node(
        package='team15_perception',
        executable='aruco_detect_publish',
        name='aruco_detect_publish',
        output='screen',
        parameters=[aruco_params_file],
        
        # ADDED: Wait for the slam_toolbox node to become available
        on_startup=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
                ),
                launch_arguments={'slam_params_file': slam_params_file, 'wait_for_slam': 'true'}.items(),
            )
        ]
    )
    
    # RVIS2
    start_rviz_node_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    #Start everything
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        start_gazebo_cmd,
        start_slam_toolbox_cmd,
        start_nav2_cmd,
        start_explore_nav_node_cmd,
        start_aruco_detect_node_cmd,
        start_rviz_node_cmd,
    ])