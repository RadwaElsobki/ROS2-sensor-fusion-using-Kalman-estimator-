import rclpy 
import serial
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan, JointState
import numpy as np 

class DistancePublisher(Node):

    def __init__(self):

        super().__init__('distance_publisher')

        self.publisher = self.create_publisher(LaserScan, 'distance', 10)
        self.state_sub = self.create_subscription(JointState, 'jekko_state', self.filter_callback, 10)
        #self.timer = self.create_timer(0.1, self.timer_callback)

        self.declare_parameter('q', 0.0)
        self.declare_parameter('r', 0.0)

        self.sensor_msg = LaserScan()

        self.sensor_msg.header.stamp = self.get_clock().now().to_msg() 
        self.sensor_msg.header.frame_id = 'distance_sensor'
        self.sensor_msg.angle_min = 0.0
        self.sensor_msg.angle_max = 0.0
        self.sensor_msg.angle_increment = 0.0
        self.sensor_msg.time_increment = 0.0
        self.sensor_msg.scan_time = 0.1  # Corresponding to the timer period
        self.sensor_msg.range_min = 0.0
        self.sensor_msg.range_max = 100.0  # You might want to set this according to your sensor's spec

        # Starting conditions
        self.x_hat = 0.6
        self.p_hat = 0.1

        # Defining the system constants
        self.f = 1.0
        self.g = 0.001
        self.h = 1.0
        self.i = 1.0

        self.sigma_p = self.get_parameter('q').get_parameter_value().double_value
        self.sigma_r = self.get_parameter('r').get_parameter_value().double_value

        self.get_logger().info(f"Using sigma P: {self.sigma_p}, sigma R: {self.sigma_r}")

        self.serial_port = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
        self.range_value = 0

    def timer_callback(self):

        self.sensor_msg.header.stamp = self.get_clock().now().to_msg()

        # Write the command to request distance data
        self.serial_port.write(b'LD\r\n')
        raw_response = self.serial_port.readline()

        if raw_response:
            try:
                
                # Decode the response and convert it to a float
                self.range_value = float(raw_response.decode('utf-8').strip())

                if self.range_value > 0.0 and self.range_value < 50.0:
                    self.sensor_msg.ranges = [self.range_value]

            except ValueError:
                self.get_logger().error(f"Received invalid data: {raw_response.decode('utf-8', errors='ignore')}")
        else:
            self.get_logger().error('No data received')

        # Publish the sensor data
        self.publisher.publish(self.sensor_msg)

    def filter_callback(self, msg):

        self.sensor_msg.header.stamp = self.get_clock().now().to_msg()

        input_value = msg.position[msg.name.index('winch_lever')]

        # Write the command to request distance data
        self.serial_port.write(b'LD\r\n')
        raw_response = self.serial_port.readline()

        if raw_response:
            try:
                # Decode the response and convert it to a float
                self.range_value = float(raw_response.decode('utf-8').strip())

                if self.range_value > 0.0 and self.range_value < 50.0:

                    # Prediction
                    self.x_hat = self.f * self.x_hat + self.g * input_value
                    self.p_hat = (self.f * self.p_hat * self.f) + np.power(self.sigma_p, 2)

                    # Update
                    k = self.p_hat * self.h * (1. / (self.h * self.p_hat * self.h + np.power(self.sigma_r, 2)))
                    self.x_hat = self.x_hat + k * (self.range_value - (self.h * self.x_hat))
                    self.p_hat = self.p_hat * (self.i - k * self.h)

                    self.sensor_msg.ranges = [self.range_value, self.x_hat]

            except ValueError:
                self.get_logger().error(f"Received invalid data: {raw_response.decode('utf-8', errors='ignore')}")
        else:
            self.get_logger().error('No data received')

        self.get_logger().info(f"Measured value: {self.range_value}, kf: {self.x_hat}")

        # Publish the sensor data
        self.publisher.publish(self.sensor_msg)


def main(args=None):

    rclpy.init(args=args)

    distance_publisher = DistancePublisher()

    try:

        rclpy.spin(distance_publisher)

    except KeyboardInterrupt:
        distance_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()