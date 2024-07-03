from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'manipulator_control'
submodules = 'manipulator_control/robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob(os.path.join('resource/*'))),
        ('lib/python3.10/site-packages/' + package_name + '/robot_control', glob(os.path.join('manipulator_control/robot_control/motionSL_hash.json')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='vinokurov1768@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot_node = manipulator_control.move_robot_node:main',
            'data_publisher = manipulator_control.data_publisher:main'
        ],
    },
)
