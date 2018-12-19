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
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from datetime import datetime
import cv2


class CameraSimulator(Node):
    """
    Main ROS Camera simulator Node function. Takes input from USB webcam and publishes a
     ROS CompressedImage and Image message to topics /iris/image and
     /iris/image

    """

    def __init__(self, filepath):
        super().__init__('camera_simulator')
        self.publisher_ = self.create_publisher(CompressedImage, '/iris/image')

        self.br = CvBridge()

        try:
            self.vc = cv2.VideoCapture(filepath)
        except:
            print("End of file")

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = ComressedImage()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        self.get_logger().info('Publishing image from simulator')
        # self.i += 1

        rval, frame = self.vc.read()

        cmprsmsg = self.br.cv2_to_compressed_imgmsg(frame)  # Convert the image to a compress message

        self.publisher_.publish(cmprsmsg)

    def get_image_msg(image):
        '''
        Get image message, takes image as input and returns CvBridge image message
        :param image: cv2 image
        :return: sensor_msgs/Imag
        '''
        msg = CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        return msg

    def get_compressed_msg(image):
        '''
        Get compressed image, takes image as inputs and returns a CompressedImage message
        :param image: cv2 image
        :return: sensor_msgs/CompressedImage
        '''
        msg = CompressedImage()
        msg.header.timestamp = str(datetime.now())
        msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        return msg


def main(args=None):
    parser = argparse.ArgumentParser(description='Video file or files to load')
    parser.add_argument('--path', type=str, default="",
                        help='path to video folder')
    parser.add_argument('--file', type=str, default="",
                        help='path to single video file')

    args = parser.parse_args()

    rclpy.init()

    camera_simulator = CameraSimulator(filepath=args.file)

    rclpy.spin(camera_simulator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
