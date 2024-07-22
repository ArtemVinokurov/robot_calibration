from setuptools import find_packages, setup
import os
import glob

package_name = 'calibration_experiment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/experiment_bringup_launch.py'])
        
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
            'collect_data_node = calibration_experiment.collect_data_node:main',
            'experiment_interface = calibration_experiment.experiment_interface:main',
            'pos_publisher = calibration_experiment.pos_publisher:main'
        ],
    },
)
