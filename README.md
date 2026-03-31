# MAV-LIWO: Multi-Axle Vehicle Lidar-Inertial-Wheel Odometry

## Overview

MAV-LIWO is a tightly-coupled multi-sensor odometry framework tailored for multi-axle steering vehicles and heavy mobile robots. 

Building upon the traditional Lidar-Inertial Odometry (LIO) paradigm, this project fuses LiDAR, IMU, and multi-axle wheel odometry within an Error-State Kalman Filter (ESKF) architecture. It is explicitly designed to process complex chassis data (independent 6-wheel speeds and 6 steering angles) to mitigate localization degradation and drift in challenging scenarios such as feature-depleted environments, long corridors, and dynamic terrains.

## Key Features

- **Multi-Axle Kinematic Modeling:** Surpasses standard dual-axle Ackermann constraints by formulating a generalized multi-axle kinematic model, processing independent velocity and steering feedback from 6 wheels (front, middle, and rear axles).
- **Advanced Motion Constraints:** Integrates Non-Holonomic Constraints (NHC) to suppress lateral/vertical drift and Zero Velocity Update (ZUPT) to eliminate stationary divergence.
- **Online State Estimation & Calibration:** Supports online joint optimization of the rotation/translation extrinsics between the IMU and the vehicle footprint, alongside wheel speed scale factors.
- **Modular Software Design:** The wheel processing logic is decoupled into a standalone `WheelProcess` C++ class, ensuring high cohesion, low computational overhead, and robust extensibility for heterogeneous chassis models.

## Custom Chassis Data Format

To handle the complex kinematics and control states of the multi-axle vehicle, a custom ROS message (`read_can`) is defined and utilized within the system. The provided dataset records the chassis state using this format:

```cpp
// Custom message structure for chassis CAN data
std_msgs/Header header
float64 read_speed_r1      // Right-front wheel speed
float64 read_speed_l1      // Left-front wheel speed
float64 read_speed_r2      // Right-middle wheel speed
float64 read_speed_l2      // Left-middle wheel speed
float64 read_speed_r3      // Right-rear wheel speed
float64 read_speed_l3      // Left-rear wheel speed
float64 read_steering_r1   // Right-front steering angle
float64 read_steering_l1   // Left-front steering angle
float64 read_steering_r2   // Right-middle steering angle
float64 read_steering_l2   // Left-middle steering angle
float64 read_steering_r3   // Right-rear steering angle
float64 read_steering_l3   // Left-rear steering angle
float64 read_phone_run     // Teleoperation: run status
float64 read_phone_gears   // Teleoperation: gear status
float64 read_phone_steering// Teleoperation: global steering command
float64 read_phone_modes   // Teleoperation: control mode
```

## Public Dataset (.bag)

To facilitate algorithm evaluation and reproducibility, we provide `MAV-NAV.bag`, a real-world ROSBag dataset collected by a multi-axle steering vehicle. The `.bag` file includes high-frequency IMU measurements, LiDAR point clouds, and the aforementioned `read_can` chassis data.

- **Link (Baidu Netdisk):** [https://pan.baidu.com/s/1v-6uBzNtCpV2VfW3_EhNsQ?pwd=8888](https://pan.baidu.com/s/1v-6uBzNtCpV2VfW3_EhNsQ?pwd=8888)
- **Extraction Code:** 8888

## Prerequisites

The framework has been developed and validated on **Ubuntu 20.04** with **ROS Noetic**.

- ROS (Robot Operating System)
- PCL (Point Cloud Library) >= 1.8
- Eigen >= 3.3.4
- C++14/17 compiler

## Build Instructions

Due to intensive template instantiations during compilation (e.g., Eigen and PCL), it is highly recommended to limit the number of parallel build jobs to prevent Out-Of-Memory (OOM) errors, especially in virtual machine environments.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/youngsterkim/MAV-LIWO.git
cd ..
catkin_make -j4
source devel/setup.bash
```

## Running the System

**1. System Configuration**
Prior to launching, ensure that the extrinsic parameters and initial covariance values in the `config/` YAML files are correctly calibrated to match your hardware setup:
- Multi-axle lever arms: `offset_T_A1_I`, `offset_T_A2_I`, `offset_T_A3_I`
- Noise covariances: `wheel_cov`, `nhc_y_cov`, `nhc_z_cov`

**2. Launch the Estimator**
```bash
roslaunch fast_lio mapping_vehicle.launch
```

**3. Play the Dataset**
In a separate terminal, play the provided `.bag` dataset:
```bash
rosbag play MAV-NAV.bag
```

## Acknowledgements

The core Lidar-Inertial processing pipeline of this project is heavily based on the foundational work of [FAST-LIO](https://github.com/hku-mars/FAST_LIO). We express our gratitude to the HKU MaRS Lab for their outstanding open-source contributions.
