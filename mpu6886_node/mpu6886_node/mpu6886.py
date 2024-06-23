import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus
import time

class MPU6886Node(Node):
    def __init__(self):
        super().__init__('mpu6886_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bus = SMBus(1)
        self.address = 0x68
        self.get_logger().info('MPU6886 Node initialized')
        self.initialize_sensor()

    def initialize_sensor(self):
        try:
            self.bus.write_byte_data(self.address, 0x6B, 0x00)  # Wake up MPU6886
            time.sleep(0.1)
            self.get_logger().info('MPU6886 sensor initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize sensor: {e}')

    def read_sensor_data(self):
        try:
            # Read accelerometer data
            accel_x = self.read_word_2c(0x3B)
            accel_y = self.read_word_2c(0x3D)
            accel_z = self.read_word_2c(0x3F)
            
            accel_scale_modifier = 16384.0
            accel_x = accel_x / accel_scale_modifier
            accel_y = accel_y / accel_scale_modifier
            accel_z = accel_z / accel_scale_modifier

            # Read gyroscope data
            gyro_x = self.read_word_2c(0x43)
            gyro_y = self.read_word_2c(0x45)
            gyro_z = self.read_word_2c(0x47)

            gyro_scale_modifier = 131.0
            gyro_x = gyro_x / gyro_scale_modifier
            gyro_y = gyro_y / gyro_scale_modifier
            gyro_z = gyro_z / gyro_scale_modifier

            self.get_logger().info(f'Read sensor data: accel_x={accel_x}, accel_y={accel_y}, accel_z={accel_z}, gyro_x={gyro_x}, gyro_y={gyro_y}, gyro_z={gyro_z}')
            return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        except Exception as e:
            self.get_logger().error(f'Failed to read sensor data: {e}')
            return None, None, None, None, None, None

    def read_word_2c(self, reg):
        try:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
            value = (high << 8) + low
            if value >= 0x8000:
                value = -((65535 - value) + 1)
            return value
        except Exception as e:
            self.get_logger().error(f'Failed to read word: {e}')
            return 0

    def timer_callback(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_sensor_data()
        if accel_x is not None and accel_y is not None and accel_z is not None and gyro_x is not None and gyro_y is not None and gyro_z is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'
            imu_msg.linear_acceleration.x = float(accel_x)
            imu_msg.linear_acceleration.y = float(accel_y)
            imu_msg.linear_acceleration.z = float(accel_z)
            imu_msg.angular_velocity.x = float(gyro_x)
            imu_msg.angular_velocity.y = float(gyro_y)
            imu_msg.angular_velocity.z = float(gyro_z)
            self.publisher_.publish(imu_msg)
            self.get_logger().info('Published IMU data')

def main(args=None):
    rclpy.init(args=args)
    node = MPU6886Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
