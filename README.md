# ROS2 Sensor Fusion Using Kalman Estimator

This repository provides a ROS2 package for sensor fusion utilizing a Kalman estimator. The package integrates data from multiple sensors to enhance the accuracy of state estimation in robotic applications.

## Features

- **Sensor Fusion**: Combines data from various sensors to improve state estimation accuracy.
- **Kalman Estimator**: Implements a Kalman filter for optimal state prediction and correction.
- **ROS2 Integration**: Designed to work seamlessly within the ROS2 framework.

## Installation

### Prerequisites

- **ROS2**: Ensure that ROS2 is installed on your system. Follow the official ROS2 installation guide for instructions.
    
- **Dependencies**: Install necessary dependencies using `rosdep`:
    
    bash
    
    CopyEdit
    
    `rosdep install --from-paths src --ignore-src -r -y`
    

### Building the Package

1. Clone this repository into your ROS2 workspace:
    
    bash
    
    CopyEdit
    
    `cd ~/ros2_ws/src git clone https://github.com/RadwaElsobki/ROS2-sensor-fusion-using-Kalman-estimator-.git`
    
2. Navigate to your workspace and build the package:
    
    bash
    
    CopyEdit
    
    `cd ~/ros2_ws colcon build`
    
3. Source the setup script:
    
    bash
    
    CopyEdit
    
    `source install/setup.bash`
    

## Usage

To launch the sensor fusion node, use the following command:

bash

CopyEdit

`ros2 launch ros2_sensor_pkg sensor_fusion_launch.py`

This will start the node responsible for fusing sensor data using the Kalman estimator.



For more information on sensor fusion and Kalman filters in ROS2, consider exploring the following resources:

- ROS2 Documentation
- [Kalman Filter for Sensor Fusion](https://github.com/sharathsrini/Kalman-Filter-for-Sensor-Fusion)
- [Sensor Fusion using Extended Kalman Filter in ROS](https://github.com/Apatil10/SensorFusion-EKF)

These resources offer valuable insights and practical examples to help you implement and understand sensor fusion using Kalman filters in ROS2.
