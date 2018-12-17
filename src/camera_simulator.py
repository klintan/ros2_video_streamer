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
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('camera_simulator')
        self.publisher_ = self.create_publisher(String, '/iris/image')
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


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
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        return msg

    def simulate_camera(path):
        '''
        Main ROS Camera simulator Node function. Takes input from USB webcam and publishes a ROS CompressedImage and Image message to
        topics hermes/sensors/webcam_image and hermes/sensors/webcam_image/compressed
        :return:
        '''
        VideoRaw = rospy.Publisher('hermes/sensors/webcam_image', Image, queue_size=1)

        VideoCompressed = rospy.Publisher('hermes/sensors/webcam_image/compressed', CompressedImage, queue_size=10)

        rospy.init_node('webcam', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        try:
            vc = cv2.VideoCapture(path)
        except:
            print("End of file")


        while not rospy.is_shutdown():
            rval, frame = vc.read()

            msg_image = get_image_msg(frame)
            msg_compressed_image = get_compressed_msg(frame)

            log_str = "Publishing image %s" % rospy.get_time()
            rospy.loginfo(log_str)
            VideoRaw.publish(msg_image)
            VideoCompressed.publish(msg_compressed_image)

            rate.sleep()



def main(args=None):

    parser = argparse.ArgumentParser(description='Video file or files to load')
    parser.add_argument('--path', type=str, default="",
                        help='path to video folder')
    parser.add_argument('--file', type=str, default="",
                        help='path to single video file')

    args = parser.parse_args()


    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()