import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Range
import csv
import os


class DistanceLoggerNode(Node):
    def __init__(self):
        super().__init__('distance_logger_node')

        # Subscribers for distances
        self.laser_subscription = self.create_subscription(LaserScan, '/distance', self.laser_callback, 10)
        self.uwb_subscription = self.create_subscription(Range, '/pozyx/filtered_distance', self.uwb_callback, 10)
        self.fused_subscription = self.create_subscription(Float32, '/fused_distance', self.fused_callback, 10)

        # Initialize variables to store the latest distances
        self.laser_distance = 0.0  # Default laser distance
        self.uwb_distance = 0.0    # Default UWB distance
        self.fused_distance = 0.0  # Default fused distance

        # Open a CSV file to write the data
        log_file_path = os.path.join(os.getcwd(), 'distance_fusion_log.csv')
        self.csv_file = open(log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write the header for the CSV file
        self.csv_writer.writerow([
            'Timestamp',
            'Laser_Distance',
            'UWB_Distance',
            'Fused_Distance'
        ])

        # Notify that the node has started
        self.get_logger().info(f"Logging data to {log_file_path}")

    def laser_callback(self, msg):
        # Update the latest laser distance (assume index 1 for relevant range)
        if len(msg.ranges) > 1:
            self.laser_distance = msg.ranges[1]
        else:
            self.laser_distance = 0.0
        # Log the data
        self.log_data()

    def uwb_callback(self, msg):
        # Update the latest UWB distance and adjust with offset if needed
        self.uwb_distance = msg.range - 0.2  # Example offset
        # Log the data
        self.log_data()

    def fused_callback(self, msg):
        # Update the latest fused distance
        self.fused_distance = msg.data
        # Log the data
        self.log_data()

    def log_data(self):
        # Combine all data and log it to the CSV file
        log_row = [
            self.get_clock().now().to_msg().sec,  # Timestamp
            self.laser_distance,
            self.uwb_distance,
            self.fused_distance
        ]
        self.csv_writer.writerow(log_row)
        self.get_logger().info(
            f"Timestamp: {log_row[0]}, Laser: {self.laser_distance:.2f}, UWB: {self.uwb_distance:.2f}, "
            f"Fused: {self.fused_distance:.2f}"
        )

    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create the distance logger node
    distance_logger_node = DistanceLoggerNode()
    
    try:
        rclpy.spin(distance_logger_node)
    except KeyboardInterrupt:
        pass
    
    # Shutdown and clean up
    distance_logger_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()