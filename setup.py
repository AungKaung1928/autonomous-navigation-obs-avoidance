from setuptools import setup
import os
from glob import glob

package_name = 'simple_navigation_project'


def recursive_files(src_dir: str) -> list:
    out = []
    for root, _dirs, files in os.walk(src_dir):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
    ] + recursive_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aung Kaung Myat',
    maintainer_email='103877301+AungKaung1928@users.noreply.github.com',
    description='Reactive patrol with obstacle avoidance and lane selection for TurtleBot3 in Gazebo Harmonic (LifecycleNode)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_controller = simple_navigation_project.patrol_controller:main',
        ],
    },
)