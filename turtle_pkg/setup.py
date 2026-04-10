from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cho',
    maintainer_email='cho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_node = turtle_pkg.pure_move:main',
            'final_astar_node = turtle_pkg.final_astar:main',
            'tembler_obj = turtle_pkg.tembler_obj:main',
            'obstacle_avoid_node = turtle_pkg.obstacle_avoid_astar:main',
            'location_node = turtle_pkg.patrol_robot.location:main',
            'gui_node = turtle_pkg.patrol_robot.gui:main',
            'patrol_robot_node = turtle_pkg.patrol_robot.patrol_robot:main',
            'person_follower = turtle_pkg.patrol_robot.person_follower:main',
            'yolo_light_node = turtle_pkg.patrol_robot.yolo_light:main',
            'yolo_human_node = turtle_pkg.patrol_robot.yolo_human:main',
            'image_debug_node = turtle_pkg.patrol_robot.image_debug:main',
            'person_detect_node = turtle_pkg.patrol_robot.person_detect:main',
            'dstar_node = turtle_pkg.test2.patrol:main'
        ],
    },
)
