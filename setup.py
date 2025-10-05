# setup.py
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'hss_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index kaydı
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # veri/launch dizinleri (glob boş dönerse hata vermez)
        (os.path.join('share', package_name, 'calib'),  glob('calib/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Gazi-Uzay',
    author_email='tayfurcnr@gmail.com',
    description='HSS Vision nodes (camera driver, vision processor).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_driver = hss_vision.camera_driver_node:main',
            'vision_processor_node = hss_vision.vision_processor_node:main',
        ],
    },
)
