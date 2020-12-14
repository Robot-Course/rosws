import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math

from geometry_msgs.msg import Twist
from gpiozero import Motor

from .encoder import Encoder


class MotorPublisher(Node):

    def __init__(self):
        super().__init__('motor_publisher')

        self.declare_parameter('pid.kp')
        self.declare_parameter('pid.ki')
        self.declare_parameter('pid.kd')
        self.declare_parameter('max_linear')
        self.declare_parameter('max_angular')
        self.declare_parameter('wheels.separation')
        self.declare_parameter('wheels.radius')
        self.declare_parameter('motor.rpm')
        self.declare_parameter('motor.precision')
        self.declare_parameter('motor.left.in1')
        self.declare_parameter('motor.left.in2')
        self.declare_parameter('motor.left.encoder.a')
        self.declare_parameter('motor.left.encoder.b')
        self.declare_parameter('motor.right.in1')
        self.declare_parameter('motor.right.in2')
        self.declare_parameter('motor.right.encoder.a')
        self.declare_parameter('motor.right.encoder.b')

        self.kp = self.get_parameter('pid.kp').get_parameter_value().double_value
        self.ki = self.get_parameter('pid.ki').get_parameter_value().double_value
        self.kd = self.get_parameter('pid.kd').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheels.separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheels.radius').get_parameter_value().double_value
        self.rpm = self.get_parameter('motor.rpm').get_parameter_value().integer_value

        self.motor_left = Motor(self.get_parameter('motor.left.in1').get_parameter_value().integer_value,
                                self.get_parameter('motor.left.in2').get_parameter_value().integer_value)
        self.motor_right = Motor(self.get_parameter('motor.right.in1').get_parameter_value().integer_value,
                                 self.get_parameter('motor.right.in2').get_parameter_value().integer_value)

        self.motor_left_encoder = Encoder(self,
                                          self.get_parameter('motor.left.encoder.a').get_parameter_value().integer_value,
                                          self.get_parameter('motor.left.encoder.b').get_parameter_value().integer_value,
                                          self.get_parameter('motor.precision').get_parameter_value().integer_value,
                                          reverse=True)
        self.motor_right_encoder = Encoder(self,
                                           self.get_parameter('motor.right.encoder.a').get_parameter_value().integer_value,
                                           self.get_parameter('motor.right.encoder.b').get_parameter_value().integer_value,
                                           self.get_parameter('motor.precision').get_parameter_value().integer_value)

        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel, QoSProfile(depth=100))

        self.timer = self.create_timer(0.1, self.pid_motor)

        self.target_l_rpm = 0
        self.target_r_rpm = 0

        self.sum_error_l = 0
        self.sum_error_r = 0
        self.last_error_l = 0
        self.last_error_r = 0

        # self.test = open('/home/ubuntu/x.txt', 'w')

    def pid_motor(self):
        error_l = self.target_l_rpm - self.motor_left_encoder.read_rpm
        self.sum_error_l = self.sum_error_l + error_l
        corrected_l = self.kp * error_l + self.ki * self.sum_error_l + self.kd * (error_l - self.last_error_l)
        self.last_error_l = error_l

        error_r = self.target_r_rpm - self.motor_right_encoder.read_rpm
        self.sum_error_r = self.sum_error_r + error_r
        corrected_r = self.kp * error_r + self.ki * self.sum_error_r + self.kd * (error_r - self.last_error_r)
        self.last_error_r = error_r
        # self.test.write('%f,%f\n' % (error_l, error_r))
        l_rpm = self.motor_left_encoder.read_rpm + corrected_l
        r_rpm = self.motor_right_encoder.read_rpm + corrected_r

        if self.target_l_rpm == 0 or l_rpm == 0:
            self.motor_left.stop()
        elif l_rpm > 0:
            self.motor_left.forward(min(abs(l_rpm) / self.rpm, 1))
        elif l_rpm < 0:
            self.motor_left.backward(min(abs(l_rpm) / self.rpm, 1))

        if self.target_r_rpm == 0 or r_rpm == 0:
            self.motor_right.stop()
        elif r_rpm > 0:
            self.motor_right.forward(min(abs(r_rpm) / self.rpm, 1))
        elif r_rpm < 0:
            self.motor_right.backward(min(abs(r_rpm) / self.rpm, 1))

    def solve_wheel_speed(self, linear, angular):
        l_vl = linear + angular * self.wheel_separation / 2.0
        r_vl = linear - angular * self.wheel_separation / 2.0

        l_speed = l_vl * 60 / (2 * math.pi * self.wheel_radius)
        r_speed = r_vl * 60 / (2 * math.pi * self.wheel_radius)

        return l_speed, r_speed

    def cmd_vel(self, msg):
        linear = max(min(self.max_linear, msg.linear.x), -self.max_linear)
        angular = max(min(self.max_angular, msg.angular.z), -self.max_angular)
        self.target_l_rpm, self.target_r_rpm = self.solve_wheel_speed(linear, angular)
        self.sum_error_l = 0
        self.sum_error_r = 0

        self.get_logger().info('[MAIN] L: %.2f, A: %.2f' % (linear, angular))
        self.get_logger().info('[SOLVED] L: %.2f, R: %.2f' % (self.target_l_rpm, self.target_r_rpm))

    def close(self):
        self.motor_left.close()
        self.motor_right.close()
        self.motor_left_encoder.close()
        self.motor_right_encoder.close()
        # self.test.close()


def main(args=None):
    rclpy.init(args=args)
    motor_publisher = MotorPublisher()
    try:
        rclpy.spin(motor_publisher)
    finally:
        motor_publisher.destroy_node()
        motor_publisher.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
