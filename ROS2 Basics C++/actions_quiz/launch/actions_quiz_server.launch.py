from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='actions_quiz',
            executable='distance_action_server_node',
            output='screen'),
    ])