from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'onrobot_rg_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gawlinski',
    maintainer_email='gawlinski.dorian@gmail.com',
    description='RG2-FT control Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'onrobot_rg_tcp_node = onrobot_rg_control.OnRobotRGTcpNode:main',
             'onrobot_rg_status_listener = onrobot_rg_control.OnRobotRGStatusListener:main',
             'onrobot_rg_simple_controller = onrobot_rg_control.OnRobotRGSimpleController:main',
             'onrobot_rg_test_node = onrobot_rg_control.TestNode:main',
             'comModbusTcp = onrobot_rg_control.comModbusTcp:main',
             'onrobot_cube = onrobot_rg_control.onrobot_cube:main',

        ],
    },
)
