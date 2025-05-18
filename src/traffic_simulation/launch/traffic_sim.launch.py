from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_simulation',
            executable='traffic_visualizer',
            name='traffic_visualizer',
            output='screen'
        )
    ])
