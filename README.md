# ROS2 Video Streamer

A ROS2 python node for streaming video files or stream images in a folder to a topic. 

Used as a camera simulator if you have pre-recorded video you want to stream, or if you have a dataset of video frames as images, that you want to replay.

#### Features:
- Stream images from folder
- Stream video from path
- Supply camera calibration file for camera or images to publish camera_info.

This is a work in progress.

## Install

### Install extra dependencies

#### Ubuntu/Debian

``` bash
apt update -y
apt install -y python3-natsort  # Installing natsort (is not in rosdep)
```

### Install other dependencies and build

``` bash
rosdep install -i --from-path . -y
colcon build --symlink-install

source ./install/setup.bash
```

## Usage

Make sure to activate the workspace where `vision_opencv` is first.

#### Run a video file

`ros2 run camera_simulator camera_simulator --type video --path <my-video-path> --loop`


#### Run a KITTI dataset folder
If for some reason you do not have a bag file of CameraInfo (camera calibration information) and
images, what this app can do is simulate this

`ros2 run camera_simulator camera_simulator --type image --path <image-folder-path> --calibration_file <calibration-path>`

In the data folder you'll find some examples of this, calibration configuration sample file (converted to ROS2 calibration file from 2009_09_08_calib.txt for the left camera (camera 1)) and some images. Start up `image_proc` as you would to rectify
your images and you should be good to go.
