from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    task_node = Node(
        package="whole_config",
        executable="exchange.py",
        output="screen"
    )
    return LaunchDescription([task_node])