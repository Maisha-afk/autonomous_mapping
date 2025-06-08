from setuptools import find_packages, setup
import os 
from glob import glob 
package_name = 'rosbot_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abd',
    maintainer_email='abd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_with_known_poses = rosbot_mapping.mapping_with_known_poses:main',
            #'autonomous_explorer = rosbot_mapping.autonomous_explorer:main',
            #'wavefront_frontier = rosbot_mapping.wavefront_frontier:main',
            'obstacle_avoider = rosbot_mapping.obstacle_avoider:main',
            'RRT_planner = rosbot_mapping.RRT_planner::main',
            'pure_pursuit = rosbot_mapping.pure_pursuit::main',
            #for cpp nodes nothing change in setup.py
        ],
    },
)
