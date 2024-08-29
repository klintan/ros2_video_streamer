# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from builtin_interfaces.msg import Time
import cv2
from cv_bridge import CvBridge
from datetime import datetime
import os
import yaml
from natsort import natsorted


class CameraSimulator(Node):
    """
    Main ROS Camera simulator Node function. Takes input from USB webcam and publishes a
     ROS CompressedImage and Image message to topics /iris/image and
     /iris/image

    """

    def __init__(self, **kwargs):
        super().__init__("camera_simulator")

        image_topic_ = self.declare_parameter("image_topic", "/image/image_raw").value
        camera_info_topic_ = self.declare_parameter("camera_info_topic", "/image/camera_info").value

        self.frame_id_ = self.declare_parameter("frame_id", "camera").value
        self.camera_name_ = self.declare_parameter("camera_name", "narrow_stereo").value

        self.calibration_file = kwargs["calibration_file"]

        self.image_publisher_ = self.create_publisher(Image, image_topic_, 5)
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, camera_info_topic_, 5)

        self.br = CvBridge()

        self.type = kwargs["type"]

        self.loop = kwargs["loop"]

        try:
            f = open(self.calibration_file)
            calib = yaml.load(f, Loader=yaml.FullLoader)
        except IOError:
            calib = None
            self.get_logger().warning(
                "Could not find calibration file " + self.calibration_file + ", will proceed without a calibration file"
            )

        if calib is not None:
            if calib["camera_name"] != self.camera_name_:
                self.get_logger().warning(
                    "["
                    + self.camera_name_
                    + "] does not match name "
                    + calib["camera_name"]
                    + " in file "
                    + self.calibration_file
                )

        self.calib = calib

        path = kwargs['path']

        if self.type == "video":
            if not os.path.isfile(path):
                raise RuntimeError(f"Invalid video path: {path}")

            try:
                self.vc = cv2.VideoCapture(kwargs["path"])
                self.vc.set(cv2.CAP_PROP_POS_MSEC, kwargs["start"])
            except:
                print("End of file")

            video_fps = self.vc.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"Publishing image with {video_fps} fps")

            self.timer = self.create_timer(1.0 / video_fps, self.image_callback)
        else:
            for image_path in natsorted(os.listdir(path), key=lambda y: y.lower()):
                if image_path.endswith(".jpg") or image_path.endswith(".jpeg") or image_path.endswith(".png"):
                    self.image_callback(os.path.join(kwargs["path"], image_path))
            self.get_logger().info("All images have been published")

    def image_callback(self, image_path=None):
        if self.type == "video":
            rval, image = self.vc.read()
            if not rval and not self.loop:
                self.get_logger().info("End of video, closing node...")
                self.timer.cancel()
                self.destroy_node()
                exit()
            elif not rval and self.loop:
                self.vc.set(cv2.CAP_PROP_POS_MSEC, 0)
                rval, image = self.vc.read()            
        elif image_path:
            image = cv2.imread(image_path)
        else:
            self.get_logger().error("Image path is none.")
            raise ValueError()

        time_msg = self.get_time_msg()
        img_msg = self.get_image_msg(image, time_msg)  # Convert the image to a message

        if self.calib:
            camera_info_msg = self.get_camera_info(time_msg)
            self.camera_info_publisher_.publish(camera_info_msg)

        self.image_publisher_.publish(img_msg)

    def get_camera_info(self, time):
        """
        From https://github.com/FurqanHabibi/cozmo_driver_ros2/blob/master/camera_info_manager.py
        :param time:
        :return:
        """
        ci = CameraInfo()

        # fill in CameraInfo fields
        ci.width = self.calib["image_width"]
        ci.height = self.calib["image_height"]
        ci.distortion_model = self.calib["distortion_model"]
        ci.d = self.calib["distortion_coefficients"]["data"]
        ci.k = self.calib["camera_matrix"]["data"]
        ci.r = self.calib["rectification_matrix"]["data"]
        ci.p = self.calib["projection_matrix"]["data"]

        ci.header.stamp = time
        ci.header.frame_id = self.frame_id_
        return ci

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def get_image_msg(self, image, time):
        """
        Get image message, takes image as input and returns CvBridge image message
        :param image: cv2 image
        :return: sensor_msgs/Imag
        """
        img_msg = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = time
        img_msg.header.frame_id = self.frame_id_
        return img_msg

    def get_compressed_msg(self, image):
        """
        Get compressed image, takes image as inputs and returns a CompressedImage message
        :param image: cv2 image
        :return: sensor_msgs/CompressedImage
        """
        msg = CompressedImage()
        msg.header.timestamp = str(datetime.now())
        msg.format = "jpeg"
        return msg


def main(args=None):
    parser = argparse.ArgumentParser(description="Video file or files to load")
    parser.add_argument("--path", type=str, default="", required=True, help="path to video folder")
    parser.add_argument("--calibration_file", type=str, default="", help="path to video folder")
    parser.add_argument("--type", type=str, default="video", help='type of "image" or "video')
    parser.add_argument("--start", type=int, default=0, help="starting position")
    parser.add_argument('--loop', action='store_true', help='loop video after end')
    parser.set_defaults(loop=False)

    extra_args, unknown = parser.parse_known_args()

    rclpy.init(args=args)

    camera_simulator = CameraSimulator(
        path=extra_args.path, type=extra_args.type, calibration_file=extra_args.calibration_file, start=extra_args.start,
        loop=extra_args.loop
    )

    rclpy.spin(camera_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
