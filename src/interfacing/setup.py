import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'interfacing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Install ALL launch/*.py
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world*')),
        (os.path.join('share', package_name, 'models', 'tb3_4walls',), glob('models/tb3_4walls/*')),
        (os.path.join('share', package_name, 'models', 'maze_0'), glob('models/maze_0/*')),
        (os.path.join('share', package_name, 'models', 'maze_1'), glob('models/maze_1/*')),
        (os.path.join('share', package_name, 'models', 'maze_2'), glob('models/maze_2/*')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='icf3ver',
    maintainer_email='icf3ver@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_forward = Interfacing.move_forward:main',
            'wallfollow = Interfacing.loop_around_wallfollow:main',
            'h1 = Interfacing.navigate_h1:main',
            'halpha = Interfacing.navigate_halpha:main',
            'hbeta = Interfacing.navigate_hbeta:main',
            'Avers = Interfacing.Aversion:main',
            'Gazebo = Interfacing.Gazebo_Follow:main',
            'FollowInside = Interfacing.Gazebo_Inside_Follow:main',
            'FollowOutside = Interfacing.Gazebo_Outside_Follow:main',
            'Gazebo_Chat = Interfacing.Gazebo_Find_Path:main',
            'Gazebo_h1 = Interfacing.Gazebo_h1:main',
        ],
    },
)
