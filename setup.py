from setuptools import setup, find_packages

package_name = 'camera_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    py_modules=[],
    zip_safe=True,
    install_requires=[
        'setuptools',
        'opencv-python'
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
