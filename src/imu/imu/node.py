import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

from gy85.adxl345.i2c import ADXL345
from gy85.hmc5883l.HMC5883L import HMC5883L
from gy85.itg3200.ITG3200 import ITG3200
from complementary_filter.complementary_filter import ComplementaryFilter


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameter('smbus')
        self.declare_parameter('adxl345.addr')
        self.declare_parameter('adxl345.data_rate')
        self.declare_parameter('adxl345.g_range')
        self.declare_parameter('adxl345.full_resolution')
        self.declare_parameter('itg3200.addr')
        self.declare_parameter('hmc5883l.addr')
        self.declare_parameter('hmc5883l.gauss')
        self.declare_parameter('hmc5883l.declination')
        orientation_stddev = self.declare_parameter('filter.orientation_stddev', 0.0).get_parameter_value().double_value
        do_bias_estimation = self.declare_parameter('filter.do_bias_estimation', True).get_parameter_value().bool_value
        do_adaptive_gain = self.declare_parameter('filter.do_adaptive_gain', True).get_parameter_value().bool_value
        gain_acc = self.declare_parameter('filter.gain_acc', 0.01).get_parameter_value().double_value
        gain_mag = self.declare_parameter('filter.gain_mag', 0.01).get_parameter_value().double_value
        bias_alpha = self.declare_parameter('filter.bias_alpha', 0.01).get_parameter_value().double_value
        self.constant_dt = self.declare_parameter('filter.constant_dt', 0.02).get_parameter_value().double_value

        self.filter = ComplementaryFilter()
        self.orientation_variance = orientation_stddev ** 2
        self.filter.setDoBiasEstimation(do_bias_estimation)
        self.filter.setDoAdaptiveGain(do_adaptive_gain)
        if not self.filter.setGainAcc(gain_acc):
            self.get_logger().warn('Invalid gain_acc passed to ComplementaryFilter.')
        if not self.filter.setGainMag(gain_mag):
            self.get_logger().warn('Invalid gain_mag passed to ComplementaryFilter.')
        if do_bias_estimation:
            if not self.filter.setBiasAlpha(bias_alpha):
                self.get_logger().warn("Invalid bias_alpha passed to ComplementaryFilter.")

        self.initialized_filter = False
        self.smbus = self.get_parameter('smbus').get_parameter_value().integer_value

        self.accelerometer = ADXL345(self.smbus, self.get_parameter('adxl345.addr').get_parameter_value().integer_value)
        self.accelerometer.set_data_rate(self.get_parameter('adxl345.data_rate').get_parameter_value().integer_value)
        self.accelerometer.set_range(self.get_parameter('adxl345.g_range').get_parameter_value().integer_value, self.get_parameter('adxl345.full_resolution').get_parameter_value().bool_value)
        self.accelerometer.power_on()
        self.accelerometer.calibrate()
        self.gyroscope = ITG3200(self.smbus, self.get_parameter('itg3200.addr').get_parameter_value().integer_value)
        self.gyroscope.calibrate()
        self.compass = HMC5883L(self.smbus, self.get_parameter('hmc5883l.addr').get_parameter_value().integer_value,
                                gauss=self.get_parameter('hmc5883l.gauss').get_parameter_value().double_value,
                                declination=self.get_parameter('hmc5883l.declination').get_parameter_value().integer_array_value)

        self.imu_publisher = self.create_publisher(Imu, 'imu', QoSProfile(depth=100))
        self.tf_publisher = TransformBroadcaster(self)
        self.timer = self.create_timer(self.constant_dt, self.poll)

    def eular_to_quaternion(self, x, y, z):
        eular = [x * math.pi / 180 / 2, y * math.pi / 180 / 2, -z * math.pi / 180 / 2]
        
        ca, cb, cc = math.cos(eular[0]), math.cos(eular[1]), math.cos(eular[2])
        sa, sb, sc = math.sin(eular[0]), math.sin(eular[1]), math.sin(eular[2])
        
        x = sa*cb*cc - ca*sb*sc
        y = ca*sb*cc + sa*cb*sc
        z = ca*cb*sc - sa*sb*cc
        w = ca*cb*cc + sa*sb*sc

        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = x, y, z, w
        return orientation

    def poll(self):
        acc = self.accelerometer.read_data(scale=True)
        gyr = self.gyroscope.read_data()
        mag = self.compass.read_data()
        self.filter.update(*acc, *gyr, *mag, self.constant_dt)

        orientation = Quaternion()
        orientation.w, orientation.x, orientation.y, orientation.z = self.filter.getOrientation()
        # roll = math.atan2(acc[1], acc[2]) * 180 / math.pi
        # pitch = math.atan2(-acc[0], math.sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180 / math.pi
        # yaw = self.compass.heading(pitch * math.pi / 180, roll * math.pi / 180)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu'
        imu.orientation = orientation
        imu.angular_velocity = Vector3(x=gyr[0], y=gyr[1], z=gyr[2])
        imu.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
        imu.orientation_covariance[0] = self.orientation_variance
        imu.orientation_covariance[1] = 0.0
        imu.orientation_covariance[2] = 0.0
        imu.orientation_covariance[3] = 0.0
        imu.orientation_covariance[4] = self.orientation_variance
        imu.orientation_covariance[5] = 0.0
        imu.orientation_covariance[6] = 0.0
        imu.orientation_covariance[7] = 0.0
        imu.orientation_covariance[8] = self.orientation_variance

        if self.filter.getDoBiasEstimation():
            imu.angular_velocity.x -= self.filter.getAngularVelocityBiasX()
            imu.angular_velocity.y -= self.filter.getAngularVelocityBiasY()
            imu.angular_velocity.z -= self.filter.getAngularVelocityBiasZ()

        self.imu_publisher.publish(imu)

        tf = TransformStamped()
        tf.transform.translation = Vector3(x=.0, y=.0, z=.0)
        tf.transform.rotation = imu.orientation
        tf.header.stamp = imu.header.stamp
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'imu'
        self.tf_publisher.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
