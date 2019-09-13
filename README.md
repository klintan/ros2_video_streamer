# ROS2 Video Streamer

A ROS2 python node for streaming video files or stream images in a folder to a topic. 

Used as a camera simulator if you have pre-recorded video you want to stream, or if you have a dataset of video frames as images, that you want to replay.

#### Features:
- Stream images from folder
- Stream video from path

#### Future features:
- Supply camera calibration file for camera or images to publish camera_info.

Could hopefully serve as a more advanced example of a ROS2 python package which has some ROS2 dependencies
such as `cv_bridge` and also some public Python libraries such as `OpenCV`.


This is a work in progress.

## Install
Create a virtual environment

`colcon build --symlink-install`

`source ./install/setup.bash`

Make sure to activate `vision_opencv` first.

## Usage
`ros2 run camera_simulator camera_simulator --type video --path <my-video-or-image-folder-path>`

