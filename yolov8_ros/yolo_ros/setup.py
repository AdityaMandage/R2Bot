from setuptools import setup
from glob import glob
import os

package_name = 'yolo_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python-headless', 'cv-bridge'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_ros.yolo_node:main',
            'segmentation_node = yolo_ros.segmentation:main'
        ],
    },
    package_data={
        '': ['msg/*.msg'],
    },
)
