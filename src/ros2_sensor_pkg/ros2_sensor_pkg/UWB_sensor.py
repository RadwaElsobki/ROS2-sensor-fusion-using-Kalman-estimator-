import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from pypozyx import PozyxSerial, DeviceRange, POZYX_SUCCESS

class PozyxDistanceNode(Node):
    def __init__(self):
        super().__init__('pozyx_distance_node')

        # Define publisher for Range message type
        self.distance_publisher = self.create_publisher(Range, '/pozyx/distance', 10)
      
        # Define a timer to periodically measure and publish distance
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

        # Initialize Pozyx device and specify the anchor ID
        self.serial_port = '/dev/ttyACM0'  # Replace with your port (e.g., COMx on Windows)
        self.anchor_id = 0x6823           # Replace with your anchor's ID

        # Initialize the Range message with static values
        self.sensor_msg = Range()
        self.sensor_msg.header.frame_id = 'UWB'
        self.sensor_msg.field_of_view = 0.1   # Example value; adjust as needed
        self.sensor_msg.radiation_type = 0     # Assuming UWB is radio-based
        self.sensor_msg.min_range = 0.0        # Minimum measurable range
        self.sensor_msg.max_range = 50.0       # Maximum measurable range

        # Initialize Pozyx device
        self.pozyx = None
        try:
            self.pozyx = PozyxSerial(self.serial_port)
            self.get_logger().info("Pozyx connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pozyx: {e}")
            self.pozyx = None  # Ensure pozyx remains None if connection fails

    def timer_callback(self):
        # Check if the pozyx object was successfully initialized
        if self.pozyx is None:
            self.get_logger().error("Pozyx device is not connected. Skipping distance measurement.")
            return

        # Create a DeviceRange object to store distance
        device_range = DeviceRange()

        try:
            # Perform ranging to get the distance between the Pozyx tag and the specified anchor
            status = self.pozyx.doRanging(self.anchor_id, device_range)
            
            if status == POZYX_SUCCESS:
                # Convert the distance from mm to meters and populate the Range message
                self.sensor_msg.header.stamp = self.get_clock().now().to_msg()  # Update timestamp
                self.sensor_msg.range = device_range.distance / 1000.0  # Convert to meters

                # Publish the populated Range message
                self.distance_publisher.publish(self.sensor_msg)
                self.get_logger().info(f"Distance to anchor {hex(self.anchor_id)}: {self.sensor_msg.range:.2f} m")
            else:
                self.get_logger().warn("Failed to measure distance.")
        except TypeError as e:
            self.get_logger().error(f"Type error in doRanging call: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in doRanging call: {e}")

def main(args=None):
    rclpy.init(args=args)
    pozyx_distance_node = PozyxDistanceNode()
    rclpy.spin(pozyx_distance_node)
    pozyx_distance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()