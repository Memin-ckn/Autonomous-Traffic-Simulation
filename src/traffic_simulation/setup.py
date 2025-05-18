from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'traffic_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 
              f'{package_name}.visualization',
              f'{package_name}.planning',
              f'{package_name}.control',
              f'{package_name}.utils',
              f'{package_name}.config',
              f'{package_name}.sensors',
              f'{package_name}.communication',
              f'{package_name}.vehicles',
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
    maintainer='Mehmet Emin Çakın',
    maintainer_email='mehmetemincakin@gmail.com',
    description='Traffic Simulation System',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_visualizer = traffic_simulation.visualization.traffic_visualizer:main',
            'collision_avoidance_node = traffic_simulation.control.collision_avoidance_node:main',
        ],
    },
)