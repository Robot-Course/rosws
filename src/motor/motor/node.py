import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
from simple_pid import PID

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty
from motor_msgs.msg import Encoder as EncoderMsg
from tf2_ros import TransformBroadcaster
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

        kp = self.get_parameter('pid.kp').get_parameter_value().double_value
        ki = self.get_parameter('pid.ki').get_parameter_value().double_value
        kd = self.get_parameter('pid.kd').get_parameter_value().double_value
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
                                          read_interval=0.3,
                                          reverse=True)
        self.motor_right_encoder = Encoder(self,
                                           self.get_parameter('motor.right.encoder.a').get_parameter_value().integer_value,
                                           self.get_parameter('motor.right.encoder.b').get_parameter_value().integer_value,
                                           self.get_parameter('motor.precision').get_parameter_value().integer_value,
                                           read_interval=0.3)

        self.cmd_vel_subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel, QoSProfile(depth=100))
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.read_imu, QoSProfile(depth=100))
        self.reset_subscription = self.create_subscription(Empty, 'reset', self.reset, QoSProfile(depth=100))
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos_profile=QoSProfile(depth=100))
        self.encoder_pub = self.create_publisher(EncoderMsg, 'encoder', qos_profile=QoSProfile(depth=100))
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(0.5, self.pid_motor)

        self.pid_l = PID(kp, ki, kd, sample_time=0.5)
        self.pid_r = PID(kp, ki, kd, sample_time=0.5)
        self.l_rpm = 0.
        self.r_rpm = 0.

        self.last_imu_time = 0.
        self.last_imu_theta = 0.
        self.odom_pose = [0., 0., 0.]

    def reset(self, _msg):
        self.odom_pose[0] = 0.
        self.odom_pose[1] = 0.
        self.odom_pose[2] = 0.

    def read_imu(self, msg):
        step_time = (self.get_clock().now().nanoseconds - self.last_imu_time) / 1e9
        self.last_imu_time = self.get_clock().now().nanoseconds

        wheel_l = self.motor_left_encoder.read_rpm * step_time / 60 * 2 * math.pi
        wheel_r = self.motor_right_encoder.read_rpm * step_time / 60 * 2 * math.pi
        delta_s = self.wheel_radius * (wheel_r + wheel_l) / 2.0
        orientation = msg.orientation
        theta = math.atan2(orientation.x*orientation.y + orientation.w*orientation.z,
                           0.5 - orientation.y*orientation.y - orientation.z*orientation.z)
        delta_theta = theta - self.last_imu_theta

        self.odom_pose[0] += delta_s * math.cos(self.odom_pose[2] + (delta_theta / 2.0))
        self.odom_pose[1] += delta_s * math.sin(self.odom_pose[2] + (delta_theta / 2.0))
        self.odom_pose[2] += delta_theta

        v = delta_s / step_time
        w = delta_theta / step_time

        self.last_imu_theta = theta

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.odom_pose[0]
        odom.pose.pose.position.y = self.odom_pose[1]
        odom.pose.pose.position.z = 0.
        odom.pose.pose.orientation = orientation
        odom.pose.covariance = [0.01, 0,    0, 0,    0,    0,
                                0,    0.01, 0, 0,    0,    0,
                                0,    0,    0, 0,    0,    0,
                                0,    0,    0, 0.04, 0,    0,
                                0,    0,    0, 0,    0.04, 0,
                                0,    0,    0, 0,    0,    0.04]

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.twist.covariance = [0.01, 0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0,
                                 0,    0, 0, 0, 0, 0.02]

        self.odom_pub.publish(odom)

        odom_tf = TransformStamped()
        odom_tf.header = odom.header
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(odom_tf)

    def pid_motor(self):
        self.l_rpm += self.pid_l(self.motor_left_encoder.read_rpm)
        self.r_rpm += self.pid_r(self.motor_right_encoder.read_rpm)

        self.encoder_pub.publish(EncoderMsg(l_rpm=self.motor_left_encoder.read_rpm,
                                            r_rpm=self.motor_right_encoder.read_rpm,
                                            l_target=self.pid_l.setpoint,
                                            r_target=self.pid_r.setpoint))

        if self.pid_l.setpoint == 0:
            self.motor_left.stop()
            self.pid_l.reset()
            self.l_rpm = 0
        elif self.l_rpm > 0:
            self.motor_left.forward(min(abs(self.l_rpm) / self.rpm, 1))
        elif self.l_rpm < 0:
            self.motor_left.backward(min(abs(self.l_rpm) / self.rpm, 1))

        if self.pid_r.setpoint == 0:
            self.motor_right.stop()
            self.pid_r.reset()
            self.r_rpm = 0
        elif self.r_rpm > 0:
            self.motor_right.forward(min(abs(self.r_rpm) / self.rpm, 1))
        elif self.r_rpm < 0:
            self.motor_right.backward(min(abs(self.r_rpm) / self.rpm, 1))

    def solve_wheel_speed(self, linear, angular):
        l_vl = linear + angular * self.wheel_separation / 2.0
        r_vl = linear - angular * self.wheel_separation / 2.0

        l_speed = l_vl * 60 / (2 * math.pi * self.wheel_radius)
        r_speed = r_vl * 60 / (2 * math.pi * self.wheel_radius)

        return l_speed, r_speed

    def cmd_vel(self, msg):
        linear = max(min(self.max_linear, msg.linear.x), -self.max_linear)
        angular = max(min(self.max_angular, msg.angular.z), -self.max_angular)
        self.pid_l.setpoint, self.pid_r.setpoint = self.solve_wheel_speed(linear, angular)

        self.get_logger().info('[CMD_VEL] L: %.2f, A: %.2f' % (linear, angular))
        self.get_logger().info('[SOLVED] L: %.2f, R: %.2f' % (self.pid_l.setpoint, self.pid_r.setpoint))

    def close(self):
        self.motor_left.close()
        self.motor_right.close()
        self.motor_left_encoder.close()
        self.motor_right_encoder.close()


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
