import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('path_planner_server'), 'config')
    
    tb3_0_planner_yaml_file = os.path.join(config_dir, "tb3_0_planner_server.yaml")
    tb3_0_controller_yaml_file = os.path.join(config_dir, "tb3_0_controller.yaml")
    tb3_0_bt_navigator_yaml_file = os.path.join(config_dir, "tb3_0_bt_navigator.yaml")
    tb3_0_recovery_yaml_file = os.path.join(config_dir, "tb3_0_recovery.yaml")

    tb3_1_planner_yaml_file = os.path.join(config_dir, "tb3_1_planner_server.yaml")
    tb3_1_controller_yaml_file = os.path.join(config_dir, "tb3_1_controller.yaml")
    tb3_1_bt_navigator_yaml_file = os.path.join(config_dir, "tb3_1_bt_navigator.yaml")
    tb3_1_recovery_yaml_file = os.path.join(config_dir, "tb3_1_recovery.yaml")

    # rviz_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz')
    # rviz_file = os.path.join(rviz_dir, "localizer_rviz_config.rviz")

    return LaunchDescription([
        Node(
            namespace='tb3_0',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[tb3_0_planner_yaml_file]),
        
        Node(
            namespace='tb3_0',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[tb3_0_controller_yaml_file]),
        
        Node(
            namespace='tb3_0',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[tb3_0_bt_navigator_yaml_file]),

        Node(
            namespace='tb3_0',
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[tb3_0_recovery_yaml_file],
            output='screen'),
        
        Node(
            namespace='tb3_1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[tb3_1_planner_yaml_file]),
        
        Node(
            namespace='tb3_1',
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[tb3_1_controller_yaml_file]),
        
        Node(
            namespace='tb3_1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[tb3_1_bt_navigator_yaml_file]),

        Node(
            namespace='tb3_1',
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[tb3_1_recovery_yaml_file],
            output='screen'),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_path_planning',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['/tb3_0/planner_server', 
                                        '/tb3_0/controller_server', 
                                        '/tb3_0/bt_navigator', 
                                        '/tb3_0/recoveries_server',
                                        '/tb3_1/planner_server', 
                                        '/tb3_1/controller_server', 
                                        '/tb3_1/bt_navigator', 
                                        '/tb3_1/recoveries_server']}])
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d' + os.path.join(rviz_dir, rviz_file)])
    ])