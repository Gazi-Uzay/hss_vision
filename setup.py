import os
from glob import glob
from setuptools import setup

package_name = 'hss_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gazi-Uzay',
    maintainer_email='tayfurcnr@gmail.com',
    description='ROS 2 package for vision processing in HSS',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = hss_vision.vision_node:main',
            'camera_publisher_node = hss_vision.camera_publisher_node:main',
        ],
    },
)
