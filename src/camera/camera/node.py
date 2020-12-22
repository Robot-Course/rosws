import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, 'camera', QoSProfile(depth=100))
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.timer = self.create_timer(1 / 60, self.poll)

    def poll(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.br.cv2_to_compressed_imgmsg(frame, 'jpg')  # Convert the image to a message
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % frame.size)

    def close(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    finally:
        camera_publisher.close()
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
