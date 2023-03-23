import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')
    
    planner_yaml_file = os.path.join(config_dir, "planner_server.yaml")
    controller_yaml_file = os.path.join(config_dir, "controller.yaml")
    bt_navigator_yaml_file = os.path.join(config_dir, "bt_navigator.yaml")
    recovery_yaml_file = os.path.join(config_dir, "recovery.yaml")

    # rviz_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz')
    # rviz_file = os.path.join(rviz_dir, "localizer_rviz_config.rviz")

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml_file]),
        
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_file]),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml_file]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml_file],
            output='screen'),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_path_planning',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['planner_server', 
                                        'controller_server', 
                                        'bt_navigator', 
                                        'recoveries_server']}])
        
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d' + os.path.join(rviz_dir, rviz_file)])
    ])