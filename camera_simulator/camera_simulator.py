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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from datetime import datetime
import cv2
import os
from natsort import natsorted, ns

class CameraSimulator(Node):
    """
    Main ROS Camera simulator Node function. Takes input from USB webcam and publishes a
     ROS CompressedImage and Image message to topics /iris/image and
     /iris/image

    """

    def __init__(self, **kwargs):
        super().__init__('camera_simulator')

        self.publisher_ = self.create_publisher(Image, '/iris/image', 5)

        self.br = CvBridge()

        self.type = kwargs["type"]

        if self.type == "video":
            try:
                self.vc = cv2.VideoCapture(kwargs["path"])
            except:
                print("End of file")

            timer_period = 0.05  # seconds
            self.timer = self.create_timer(timer_period, self.image_callback)
        else:
            for image_path in natsorted(os.listdir(kwargs["path"]), key=lambda y: y.lower()):
                if image_path.endswith(".jpg") or image_path.endswith(".jpeg") or image_path.endswith(".png"):
                    self.image_callback(os.path.join(kwargs["path"], image_path))
            rclpy.get_logger().info("All images have been published")


    def image_callback(self, image_path=None):
        self.get_logger().info('Publishing image from simulator')

        if self.type == "video":
            rval, image = self.vc.read()
        elif image_path:
            image = cv2.imread(image_path)
        else:
            rclpy.get_logger().error("Image path is none.")
            raise ValueError()

        img_msg = self.br.cv2_to_imgmsg(image)  # Convert the image to a message

        #img_msg.header.stamp = self.get_clock().now()
        img_msg.header.frame_id = "camera"

        self.publisher_.publish(img_msg)

    def get_image_msg(self, image):
        '''
        Get image message, takes image as input and returns CvBridge image message
        :param image: cv2 image
        :return: sensor_msgs/Imag
        '''
        msg = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        return msg

    def get_compressed_msg(self, image):
        '''
        Get compressed image, takes image as inputs and returns a CompressedImage message
        :param image: cv2 image
        :return: sensor_msgs/CompressedImage
        '''
        msg = CompressedImage()
        msg.header.timestamp = str(datetime.now())
        msg.format = "jpeg"
        return msg


def main(args=None):
    parser = argparse.ArgumentParser(description='Video file or files to load')
    parser.add_argument('--path', type=str, default="", required=True,
                        help='path to video folder')
    parser.add_argument('--type', type=str, default="video",
                        help='type of "image" or "video')

    extra_args = parser.parse_args()

    rclpy.init(args=args)

    camera_simulator = CameraSimulator(path=extra_args.path, type=extra_args.path)

    rclpy.spin(camera_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
