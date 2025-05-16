from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Generate launch description for the traffic simulation."""
    
    # Declare launch arguments
    visualizer_type_arg = DeclareLaunchArgument(
        'visualizer_type',
        default_value='refactored',
        description='Type of visualizer to use (refactored, simple, or updated)'
    )
    
    # Create appropriate node based on visualizer type
    visualizer_node = Node(
        condition=LaunchConfiguration('visualizer_type').equals('refactored'),
        package='traffic_simulation',
        executable='refactored_visualizer',
        name='refactored_visualizer',
        output='screen'
    )
    
    simple_visualizer_node = Node(
        condition=LaunchConfiguration('visualizer_type').equals('simple'),
        package='traffic_simulation',
        executable='simple_visualizer',
        name='simple_visualizer',
        output='screen'
    )
    
    updated_visualizer_node = Node(
        condition=LaunchConfiguration('visualizer_type').equals('updated'),
        package='traffic_simulation',
        executable='simple_visualizer_updated',
        name='simple_visualizer_updated',
        output='screen'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(visualizer_type_arg)
    
    # Add nodes
    ld.add_action(visualizer_node)
    ld.add_action(simple_visualizer_node)
    ld.add_action(updated_visualizer_node)
    
    return ld 