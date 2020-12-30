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
        self.dt = self.declare_parameter('dt').get_parameter_value().double_value

        self.beta = 1  # math.sqrt(3 / 4) * math.pi * (10 / 180)
        self.orientation = Quaternion(w=1.0)

        self.smbus = self.get_parameter('smbus').get_parameter_value().integer_value

        self.accelerometer = ADXL345(self.smbus, self.get_parameter('adxl345.addr').get_parameter_value().integer_value)
        self.accelerometer.set_data_rate(self.get_parameter('adxl345.data_rate').get_parameter_value().integer_value)
        self.accelerometer.set_range(self.get_parameter('adxl345.g_range').get_parameter_value().integer_value, self.get_parameter('adxl345.full_resolution').get_parameter_value().bool_value)
        self.accelerometer.power_on()
        self.get_logger().info('Calibrating accelerometer...')
        self.accelerometer.calibrate()
        self.get_logger().info('Calibrated accelerometer.')
        self.gyroscope = ITG3200(self.smbus, self.get_parameter('itg3200.addr').get_parameter_value().integer_value)
        self.get_logger().info('Calibrating gyroscope...')
        self.gyroscope.calibrate()
        self.get_logger().info('Calibrated gyroscope.')
        self.get_logger().info('%f, %f, %f' % (self.gyroscope.offset_x, self.gyroscope.offset_y, self.gyroscope.offset_z))
        self.compass = HMC5883L(self.smbus, self.get_parameter('hmc5883l.addr').get_parameter_value().integer_value,
                                gauss=self.get_parameter('hmc5883l.gauss').get_parameter_value().double_value,
                                declination=self.get_parameter('hmc5883l.declination').get_parameter_value().integer_array_value)

        self.imu_publisher = self.create_publisher(Imu, 'imu', QoSProfile(depth=100))
        self.tf_publisher = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.poll)

    def poll(self):
        acc = self.accelerometer.read_data(scale=True)
        gyr = self.gyroscope.read_data(rads=True)
        mag = self.compass.read_data()

        self.orientation = self.MadgwickQuaternionUpdate(*acc, *gyr, *mag)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu'
        imu.orientation = self.orientation
        imu.angular_velocity = Vector3(x=gyr[0], y=gyr[1], z=gyr[2])
        imu.linear_acceleration = Vector3(x=acc[0], y=acc[1], z=acc[2])
        self.imu_publisher.publish(imu)

        tf = TransformStamped()
        tf.transform.translation = Vector3(x=.0, y=.0, z=.0)
        tf.transform.rotation = imu.orientation
        tf.header.stamp = imu.header.stamp
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'imu'
        self.tf_publisher.sendTransform(tf)

    def MadgwickQuaternionUpdate(self, ax, ay, az, gx, gy, gz, mx, my, mz):
        q1, q2, q3, q4 = self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z

        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:  # handle NaN
            return self.orientation
        norm = 1.0 / norm
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0.0:  # handle NaN
            return self.orientation
        norm = 1.0 / norm
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)  # normalise step magnitude
        norm = 1.0 / norm
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        q1 += qDot1 * self.dt
        q2 += qDot2 * self.dt
        q3 += qDot3 * self.dt
        q4 += qDot4 * self.dt
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        norm = 1.0 / norm
        return Quaternion(w=q1 * norm, x=q2 * norm, y=q3 * norm, z=q4 * norm)


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
