import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera', QoSProfile(depth=100))
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def loop(self):
        ret, frame = self.cap.read()
        msg = self.br.cv2_to_imgmsg(frame)  # Convert the image to a message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % frame.size)


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        camera_publisher.loop()
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
