import os
import glob
from setuptools import setup

_RESOURCE_DIR = os.path.join('pcd_publisher', 'resource')


def _resource_data_files():
    result = []
    for f in glob.glob(os.path.join(_RESOURCE_DIR, '*')):
        if os.path.isfile(f):
            rel = os.path.relpath(f, _RESOURCE_DIR)
            dest_dir = os.path.join('share', 'pcd_publisher', 'resource')
            result.append((dest_dir, [f]))
    return result


setup(
    name='pcd_publisher',
    version='0.1.0',
    packages=['pcd_publisher'],
    package_data={'pcd_publisher': ['resource/*.pcd']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pcd_publisher']),
    ] + _resource_data_files(),
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'open3d'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'pcd_publisher_node = pcd_publisher.pcd_publisher_node:main',
        ],
    },
)
