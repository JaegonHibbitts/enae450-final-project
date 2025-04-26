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
            'move_forward = interfacing.move_forward:main',
            'navigate = interfacing.navigate:main',
            'wallfollow = interfacing.loop_around_wallfollow:main'
        ],
    },
)
