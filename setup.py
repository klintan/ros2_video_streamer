from setuptools import setup

setup(
    name='camera_simulator',
    version='0.0.1',
    packages=["src"],
    py_modules=[],
    install_requires=['setuptools'],
    author='Andreas Klintberg',
    maintainer='Andreas Klintberg',
    keywords=['ROS2'],
    description='Camera simulator - run a recorded video file streamed on a ros topic as if a live webcamera.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'camera_simulator = src.camera_simulator:main',
        ],
    },
)