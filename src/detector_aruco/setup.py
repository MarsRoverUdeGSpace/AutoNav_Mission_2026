import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maya_aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fer',
    maintainer_email='fernando.alcaraz@inbest.cloud',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detector_node = maya_aruco_detector.aruco_detector_node:main',
            'camera_publisher_node = maya_aruco_detector.camera_publisher_node:main',
            'screenshot_node = maya_aruco_detector.screenshot_node:main',
        ],
    },
)
