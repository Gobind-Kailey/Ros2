from setuptools import find_packages, setup

package_name = 'ros2_motion_python'

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
    maintainer='Gobind Kailey',
    maintainer_email='gobind2975@gmail.com',
    description='Learn about ROS2 motion',
    license='GNU',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # name of the script = package name.module(file) name:function name
            'mover=ros2_motion_python.simple_turtlesim_motion:main',
        ],
    },
)
