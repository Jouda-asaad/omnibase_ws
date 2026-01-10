from setuptools import setup
import os
from glob import glob

package_name = 'omnibase_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Control nodes for omnibase robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_teleop = omnibase_control.uav_teleop:main',
            'drone_gui = omnibase_control.drone_gui:main',
            'force_controller = omnibase_control.force_controller:main',
            'custom_planar_move = omnibase_control.custom_planar_move:main',
        ],
    },
)
