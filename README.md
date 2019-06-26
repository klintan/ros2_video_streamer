# ROS2 Video Streamer

A ROS2 python node for streaming video files to a topic. 
Used as a camera simulator if you have pre-recorded video you want to stream.

Could hopefully server as a more advanced example of a ROS2 python package which has some ROS2 dependencies
such as `cv_bridge` and also some public Python libraries such as `OpenCV`.


This is a work in progress.

## Install
Create a virtual environment

`virtualenv3 .`

`activate`

`colcon build --symlink-install`


## Usage
`ros2 run camera_simulator camera_simulator`

