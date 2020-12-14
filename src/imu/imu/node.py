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
        self.timer = self.create_timer(0.02, self.poll)

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
        acc = self.accelerometer.read_data()
        gyr = self.gyroscope.read_data()

        roll = math.atan2(acc[1], acc[2]) * 180 / math.pi
        pitch = math.atan2(-acc[0], math.sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * 180 / math.pi
        yaw = self.compass.heading()

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu'
        imu.orientation = self.eular_to_quaternion(roll, pitch, yaw)
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
