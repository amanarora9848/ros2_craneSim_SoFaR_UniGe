import os
from glob import glob
from setuptools import setup

package_name = 'ros2_craneSim_SoFaR_UniGe'
lib = 'ros2_craneSim_SoFaR_UniGe/lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, lib],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "resource"), glob("resource/*.png")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amanarora9848',
    maintainer_email='aman.arora9848@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crane_sim_node = ros2_craneSim_SoFaR_UniGe.crane_sim_node:main', 
            'motor_x_controller = ros2_craneSim_SoFaR_UniGe.motor_x_controller:main',
            'motor_y_controller = ros2_craneSim_SoFaR_UniGe.motor_y_controller:main',
            'robot_logic = ros2_craneSim_SoFaR_UniGe.robot_logic:main',
        ],
    },
)
