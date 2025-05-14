from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_simulation',
            executable='simple_visualizer',
            name='simple_visualizer',
            output='screen'
        )
    ])
