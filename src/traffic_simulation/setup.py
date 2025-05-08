from setuptools import setup, find_packages

package_name = 'traffic_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['traffic_simulation/config/default_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/traffic_sim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 traffic simulation with Pygame visualization',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = traffic_simulation.visualization.gui:main',
            'path_planner_node = traffic_simulation.planning.path_planner:main',
            'path_follower_node = traffic_simulation.control.path_follower:main',
            'route_planner_node = traffic_simulation.planning.route_planner:main',
            'vel_subscriber_node = traffic_simulation.control.velocity_subscriber:main',
            'collision_avoidance_node = traffic_simulation.control.collision_avoidance:main',
        ],
    },
)