import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheel_leg_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='1760227036@qq.com',
    description='Wheel-leg robot LQR + VMC balance controller (ROS2 node)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_leg_controller = wheel_leg_controller.wheel_leg_controller:main',
        ],
    },
)
