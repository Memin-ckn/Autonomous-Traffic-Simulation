from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'traffic_simulation'

setup(
    name=package_name,
    version='0.0.1',
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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='memin',
    maintainer_email='memin@todo.todo',
    description='Traffic simulation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = traffic_simulation.visualization.gui:main',
            'simple_visualizer = traffic_simulation.visualization.simple_visualizer:main',
        ],
    },
)