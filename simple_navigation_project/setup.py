from setuptools import setup

package_name = 'simple_navigation_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2_simple.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@email.com',
    description='Simple Nav2 navigation demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoider = simple_navigation_project.obstacle_avoider:main'
        ],
    },
)
