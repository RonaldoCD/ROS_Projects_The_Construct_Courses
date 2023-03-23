import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    maps_dir = os.path.join(get_package_share_directory('map_server'), 'maps')
    map_file = os.path.join(maps_dir, "turtlebot_area_multi.yaml")
    
    config_dir = os.path.join(get_package_share_directory('localization_server'), 'config')
    tb3_0_amcl_yaml_file = os.path.join(config_dir, "tb3_0_amcl_config.yaml")
    tb3_1_amcl_yaml_file = os.path.join(config_dir, "tb3_1_amcl_config.yaml")

    # rviz_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz')
    # rviz_file = os.path.join(rviz_dir, "localizer_rviz_config.rviz")

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename':map_file}]),
        
        Node(
            namespace='tb3_0',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[tb3_0_amcl_yaml_file]),
        
        Node(
            namespace='tb3_1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[tb3_1_amcl_yaml_file]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server',
                                        'tb3_0/amcl',
                                        'tb3_1/amcl']}])
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d' + os.path.join(rviz_dir, rviz_file)])
    ])