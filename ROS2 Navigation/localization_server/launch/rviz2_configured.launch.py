import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    rviz_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz')
    rviz_file = os.path.join(rviz_dir, "localizer_rviz_config.rviz")

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(rviz_dir, rviz_file)])
    ])