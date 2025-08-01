from setuptools import setup
import os
from glob import glob

package_name = 'simple_navigation_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Use glob to automatically find launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*launch.py')),
        # Use glob to automatically find config files  
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@email.com', 
    description='Simple Nav2 navigation demo with TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoider = simple_navigation_project.obstacle_avoider:main'
        ],
    },
)
