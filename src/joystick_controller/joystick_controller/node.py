import rclpy
import evdev
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist


class JoystickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')

        self.declare_parameter('device')
        self.declare_parameter('max_linear')
        self.declare_parameter('max_angular')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', QoSProfile(depth=100))

        self.device = None
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if device.name == self.get_parameter('device').get_parameter_value().string_value:
                self.device = device
            else:
                device.close()

        assert self.device is not None, '"%s" is not found.' % self.get_parameter('device').get_parameter_value().string_value

        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value

    def loop(self):
        msg = Twist()
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_HAT0Y:
                    msg.linear.x = self.max_linear * -event.value
                elif event.code == evdev.ecodes.ABS_HAT0X:
                    msg.angular.z = self.max_angular * event.value

                elif event.code == evdev.ecodes.ABS_Y:
                    if event.value > 5000 or event.value < -5000:
                        msg.linear.x = -self.max_linear * (abs(event.value) / event.value) * ((abs(event.value) - 5000) / 27767)
                    else:
                        msg.linear.x = 0.0
                elif event.code == evdev.ecodes.ABS_X:
                    if event.value > 5000 or event.value < -5000:
                        msg.angular.z = self.max_angular * (abs(event.value) / event.value) * ((abs(event.value) - 5000) / 27767)
                    else:
                        msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.get_logger().info('Publishing: x=%f, z=%f' % (msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    joystick_publisher = JoystickPublisher()
    try:
        joystick_publisher.loop()
    finally:
        joystick_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
