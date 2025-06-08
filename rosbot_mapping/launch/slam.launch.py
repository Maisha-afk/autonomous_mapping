import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",  # Changed from true to false unless using Gazebo
        description="Use simulation (Gazebo) clock if true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("rosbot_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to the SLAM configuration file"
    )

    # Node for SLAM Toolbox - Changed to async and added lifecycle
    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",  # Changed from sync to async
        name="slam_toolbox",
        namespace="",
        output="screen",
        arguments=['--ros-args', '--log-level', 'debug'],  # Add debug logging
        parameters=[
            LaunchConfiguration("slam_config"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "base_frame": "base_link",
                "map_frame": "map",
                "odom_frame": "odom",
                "debug_logging": True  # Enable internal debug logging
            }
        ]
    )

    # Map Saver Node
    map_saver_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        namespace="",
        output='screen',
        parameters=[
            {'save_map_timeout': 5.0},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'free_thresh_default': 0.196},
            {'occupied_thresh_default': 0.65},
            {'save_map_on_exit': True}  # Add this parameter
        ]
    )

    # Lifecycle Manager Node - Fixed node names
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        namespace="",
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['slam_toolbox', 'map_saver_server']},  # Fixed node list
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    rviz_config_path = os.path.join(
        get_package_share_directory("rosbot_mapping"),
        "rviz",
        "mapping.rviz"
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    #obstacle avoider node:
    obstacle_avoider = Node(
        package='rosbot_mapping',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        namespace='',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('rosbot_mapping'),
            'config',
            'obstacle_avoider.yaml'
    )]
)


    
    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox,
        map_saver_server,
        lifecycle_manager,
        rviz_node,
        
    ])