from setuptools import setup, find_packages
import os
from glob import glob

PACKAGE_NAME = 'camera_simulator'
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml"))),
    ],
    py_modules=[],
    zip_safe=True,
    install_requires=[
        'setuptools',
        'opencv-python',
        'natsort'
    ],
    author='Andreas Klintberg',
    maintainer='Andreas Klintberg',
    keywords=['ROS2'],
    description='Camera simulator - run a recorded video file streamed on a ros topic as if a live webcamera.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'camera_simulator = camera_simulator.camera_simulator:main',
        ],
    },
)
