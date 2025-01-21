import rclpy
import message_filters
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from sensor_msgs.msg import JointState, LaserScan
import numpy as np

class KalmanFilter2D:
    def __init__(self, Q, R, B):
        # State transition matrix F (assumes constant state between updates)
        self.F = np.eye(2)

        # Process noise covariance Q (uncertainty in distance)
        self.Q = Q

        # Measurement matrix H (maps state to the measurement)
        self.H = np.array([[1, 0],
                           [1, 0]])

        # Measurement noise covariance R for laser and UWB
        self.R = R

        # Control matrix B (influence of boom_extension)
        self.B = B

        # Initial state estimate (distance and boom extension) and covariance
        self.x_est = np.zeros((2, 1))  # Initial distance and boom extension
        self.P_est = np.eye(2)  # Initial uncertainty

    def predict(self, boom_extension):
        # Prediction step with control input for boom_extension effect
        u = np.array([[boom_extension]])  # Control input
        self.x_est = self.F @ self.x_est + self.B @ u  # Prediction with control effect
        self.P_est = self.F @ self.P_est @ self.F.T + self.Q

    def update(self, z):
        # Compute Kalman gain
        S = self.H @ self.P_est @ self.H.T + self.R
        K = self.P_est @ self.H.T @ np.linalg.inv(S)

        # Update estimate with measurement
        y = z - self.H @ self.x_est  # Measurement residual
        self.x_est = self.x_est + K @ y

        # Update estimate covariance
        self.P_est = (np.eye(2) - K @ self.H) @ self.P_est

        # Return the updated distance (ignoring boom_extension as part of measurement update)
        return self.x_est[0, 0]

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Define process and measurement noise covariances
        Q = np.array([[0.01, 0], [0, 0.01]])  # Process noise covariance
        R = np.array([[0.2, 0], [0, 0.07]])  # Measurement noise for laser and UWB
        B = np.array([[0.05], [0.0]])        # Control gain (affect of boom_extension on distance)

        # Initialize Kalman filter with process, measurement noise, and control gain
        self.kalman_filter = KalmanFilter2D(Q, R, B)

        # Subscribers for laser (LaserScan) and UWB (Float32) estimated distances and Winch_lever (JointState)
        self.laser_subscription = message_filters.Subscriber(self, LaserScan, '/distance')
        self.uwb_subscription = message_filters.Subscriber(self, Range, '/pozyx/filtered_distance')
        self.winch_lever_subscription = self.create_subscription(JointState, 'jekko_state', self.winch_lever_callback, 10)
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.laser_subscription, self.uwb_subscription], 10, 1.0, allow_headerless = False)
        self.time_synchronizer.registerCallback(self.try_fuse_data)

        # Publisher for fused distance
        self.fused_distance_publisher = self.create_publisher(Float32, '/fused_distance', 10)

        # Latest estimates from each sensor
        self.winch_lever = 0.0  # Default value if no boom data received

    def winch_lever_callback(self, msg):
        self.winch_lever = msg.position[msg.name.index('winch_lever')]
        
    def try_fuse_data(self, laser_distance_msg, uwb_distance_msg):

        laser_distance = laser_distance_msg.ranges[1]  # or choose an index based on your needs
        uwb_distance = ((uwb_distance_msg).range) - 0.2  # (-0.2) because of hardware installation difference on crane 

        # Create measurement vector with laser and UWB distances
        z = np.array([[laser_distance], [uwb_distance]])

        # Predict step using boom_extension as control input
        self.kalman_filter.predict(self.winch_lever)

        # Update step with the measurement vector z
        distance = self.kalman_filter.update(z) 

        # Publish the fused distance only
        distance_msg = Float32()
        distance_msg.data = distance
        self.fused_distance_publisher.publish(distance_msg)

        # Log the result (distance only)
        self.get_logger().info(f"Fused Distance: {distance:.2f} m")

def main(args=None):
    rclpy.init(args=args)
    
    # Create the sensor fusion node
    sensor_fusion_node = SensorFusionNode()
    
    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    
    # Shutdown and clean up
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()