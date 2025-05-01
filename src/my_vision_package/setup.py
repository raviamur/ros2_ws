from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_vision_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ravi',
    maintainer_email='ravi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ "time_synchronizer = my_vision_package.time_synchronizer:main",
                            'camera_info_republisher = my_vision_package.camera_info_republisher:main',
                            'republish_sync = my_vision_package.republish_sync:main',
                            'stereo_cluster_node = my_vision_package.stereo_cluster_node:main',
                            'wls_publish_node = my_vision_package.wls_publish_node:main',
                            'zone_based_wls_filter_node = my_vision_package.zone_based_wls_filter_node:main',
                            'stereo_cluster_node_wls = my_vision_package.stereo_cluster_node_wls:main',
                            'right_camera_sync_republisher = my_vision_package.right_camera_sync_republisher:main',
                            'zone_wls_ack_node = my_vision_package.zone_wls_ack_node:main',
                            'ros2_serial_bridge_node = my_vision_package.ros2_serial_bridge_node:main',
                            'wls_filter_node = my_vision_package.wls_filter_node:main',
        ],

    },
)
