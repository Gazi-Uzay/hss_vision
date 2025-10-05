from setuptools import setup

package_name = 'hss_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/camera_profiles.yaml']),
        ('share/' + package_name + '/calib',  ['hss_vision/calib/internal.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Gazi-Uzay',
    author_email='tayfurcnr@gmail.com',
    description='Camera driver node (OpenCV + cv_bridge) publishing images and custom HSS CameraInfo.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_driver = hss_vision.camera_driver_node:main',
        ],
    },
)