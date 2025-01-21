import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/radwa/Documents/Sensor_Fusion_Kalman_Estimator_ROS2/install/ros2_sensor_pkg'
