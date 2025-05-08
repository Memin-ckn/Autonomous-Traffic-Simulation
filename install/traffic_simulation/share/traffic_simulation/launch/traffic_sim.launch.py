from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_simulation',
            executable='gui_node',
            name='gui_node',
            output='screen'
        ),
        Node(
            package='traffic_simulation',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        Node(
            package='traffic_simulation',
            executable='path_follower_node',
            name='path_follower_node',
            output='screen'
        ),
        Node(
            package='traffic_simulation',
            executable='collision_avoidance_node',
            name='collision_avoidance_node',
            output='screen'
        )
    ])
