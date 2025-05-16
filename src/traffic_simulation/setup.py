from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'traffic_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 
              f'{package_name}.visualization',
              f'{package_name}.visualization.components',
              f'{package_name}.planning',
              f'{package_name}.control',
              f'{package_name}.utils',
              f'{package_name}.config',
              f'{package_name}.sensors',
              f'{package_name}.communication',
              f'{package_name}.vehicles',
              f'{package_name}.unused',
              'tests'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), 
         glob('traffic_simulation/models/*.png')),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='Traffic Simulation Team',
    maintainer_email='team@trafficsim.org',
    description='ROS2 traffic simulation with component-based architecture',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = traffic_simulation.visualization.gui:main',
            'simple_visualizer = traffic_simulation.visualization.simple_visualizer:main',
            'simple_visualizer_updated = traffic_simulation.visualization.simple_visualizer_updated:main',
            'refactored_visualizer = traffic_simulation.visualization.refactored_visualizer:main',
            'collision_avoidance_node = traffic_simulation.control.collision_avoidance_node:main',
        ],
    },
)