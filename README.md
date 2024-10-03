# Robot Math Utils

A C++ maths library for robotics applications, providing a collection of utility functions for robot kinematics, transformations, and control. This library is inspired by the Modern Robotics book codebase and includes additional functionalities for data logging, numerical computations, and robot controller functions.

## Table of Contents

- [Robot Math Utils](#robot-math-utils)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
    - [Under development](#under-development)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
      - [Dependencies from `CMakeLists.txt`:](#dependencies-from-cmakeliststxt)
  - [Functions Overview](#functions-overview)
    - [Data Logging](#data-logging)
    - [Numerical Conditions](#numerical-conditions)
    - [Basic Maths Functions](#basic-maths-functions)
    - [Robot Transformation Functions](#robot-transformation-functions)
      - [Homogeneous Coordinates](#homogeneous-coordinates)
      - [Image Coordinates](#image-coordinates)
    - [SO(3) and so(3) Functions](#so3-and-so3-functions)
    - [SE(3) and se(3) Functions](#se3-and-se3-functions)
      - [Pose Conversions](#pose-conversions)
    - [Pose Preprocessing](#pose-preprocessing)
    - [Robot Controller Functions](#robot-controller-functions)

## Features

- **Data Logging**: Functions to initialise and log data to CSV files.
- **Numerical Computations**: Utilities for near-zero checks, moving averages, and statistical calculations.
- **Maths Utilities**: Basic mathematical functions, including error percentage, normalisation, and random number generation.
- **Robot Transformations**: Functions for homogeneous coordinates, rotations, quaternions, and pose transformations.
- **Kinematics**: Implementations of exponential and logarithmic mappings for rotations and transformations.
- **Control Functions**: Utilities for error thresholds, S-curve generation, and proportional controller.
- **Eigen Library Integration**: Built on top of the Eigen library for linear algebra operations.
  
### Under development
- **Kinematics**: D-H parameters, forward and inverse kinematics are still under development.

## Installation

This repository is ROS 2-based and utilises several dependencies that are common in robotics applications. Follow the instructions below to build the package in a ROS 2 environment.

### Prerequisites

Make sure you have the following installed:

- **ROS 2 humble**: Installed and properly sourced. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) if you haven't set it up yet.
- **C++17 Compiler**: Ensure your compiler supports C++17.
- **Dependencies**: The package depends on several libraries and ROS 2 packages.

#### Dependencies from `CMakeLists.txt`:

- `Eigen3`
- `PCL` (Point Cloud Library): `common`, `io`, `registration`, `visualisation`, `surface`, `filters`, `segmentation`
- `OpenCV`
- `Threads`
- ROS 2 packages:
  - `rclcpp`
  - `rclcpp_action`
  - `rclcpp_lifecycle`
  - `ament_index_cpp`
  - `pluginlib`
  - `std_msgs`
  - `geometry_msgs`
  - `class_loader`
  - `sensor_msgs`
  - `shape_msgs`
  - `tf2`
  - `tf2_geometry_msgs`
  - `tf2_ros`
  - `tf2_msgs`
  - `visualisation_msgs`
  - `lifecycle_msgs`
  - `pcl_conversions`
  - `cv_bridge`
  - `image_transport`
  - `vision_msgs`
  - `std_srvs`
- System libraries:
  - `SDL2`
  - `Boost` (specifically `system` component)

Clone this repo into your ROS 2 workspace, install dependencies, and build pkg:

```bash
cd ~/ros2_ws/src && git clone https://github.com/your_username/robot_math_utils.git

cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y

cd ~/ros2_ws && colcon build --packages-select robot_math_utils && . install/setup.bash
```

Run the ROS 2 node for test
```bash
cd ~/ros2_ws && . install/setup.bash
ros2 run robot_math_utils robot_math_utils_test
```

## Functions Overview

### Data Logging

- `PrintVec`: Print an Eigen vector with a name.
- `InitDatalog`: Initialise a CSV file for data logging.
- `Datalog`: Log data vectors to a CSV file with timestamp and index.

### Numerical Conditions

- `NearZero`: Check if a value is near zero within a threshold.
- `MeanBuffer`: Compute the mean of vectors in a buffer.
- `StdBuffer`: Compute the standard deviation of vectors in a buffer.
- `MAvg`: Compute the moving average of vectors.

### Basic Maths Functions

- `ErrorPercentage`: Calculate the error percentage between measurement and ground truth.
- `Sinc`: Compute the sinc function.
- `Norm`: Compute the norm of a vector.
- `MatNorm`: Compute the norm of a matrix.
- `Normalized`: Normalise a vector.
- `Inv`: Compute the inverse of a matrix.
- `Transpose`: Compute the transpose of a matrix.
- `Det`: Compute the determinant of a matrix.
- `Tr`: Compute the trace of a matrix.
- `ConstrainedAngle`: Constrain an angle within a range.
- `random_normal`: Generate a random number from a normal distribution.

### Robot Transformation Functions

#### Homogeneous Coordinates

- `R3Vec2Homo`: Convert a 3D vector to homogeneous coordinates.
- `Homo2R3Vec`: Convert homogeneous coordinates to a 3D vector.

#### Image Coordinates

- `ImgCoord2Homo`: Convert image coordinates to homogeneous form.
- `Homo2ImgCoord`: Convert homogeneous image coordinates back to image coordinates.

### SO(3) and so(3) Functions

- `R3Vec2so3Mat`: Convert a 3D vector to an so(3) matrix.
- `so3Mat2R3Vec`: Convert an so(3) matrix back to a 3D vector.
- `AxisAng3`: Convert a so(3) vector to axis-angle representation.
- `so32Quat`: Convert a so(3) vector to a quaternion.
- `Quat2so3`: Convert a quaternion to a so(3) vector.
- `InvQuat`: Compute the inverse of a quaternion.
- `QuatMul`: Multiply two quaternions.
- `TransformQuats`: Apply a sequence of quaternion transformations.
- `MatrixExp3`: Compute the matrix exponential for rotations.
- `MatrixLog3`: Compute the matrix logarithm for rotations.
- `Rotx`, `Roty`, `Rotz`: Generate rotation matrices about the x, y, z axes.
- `Rotxyz`, `Rotzyx`: Generate rotation matrices from Euler angles.
- `Rot2zyxEuler`: Convert a rotation matrix to ZYX Euler angles.
- `Quatx`, `Quaty`, `Quatz`: Generate quaternions for rotations about x, y, z axes.
- `zyxEuler2Quat`: Convert ZYX Euler angles to a quaternion.
- `Quat2zyxEuler`: Convert a quaternion to ZYX Euler angles.
- `RotateR3VecFromQuat`: Rotate a 3D vector using a quaternion.

### SE(3) and se(3) Functions

- `R6Vec2se3Mat`: Convert a 6D vector to an se(3) matrix.
- `se3Mat2R6Vec`: Convert an se(3) matrix back to a 6D vector.
- `AxisAng6`: Convert an se(3) vector to axis-angle representation.
- `MatrixExp6`: Compute the matrix exponential for transformations.
- `MatrixLog6`: Compute the matrix logarithm for transformations.

#### Pose Conversions

- `PosQuat2R6Pose`, `R6Pose2PosQuat`: Convert between position-quaternion and 6D pose representations.
- `PosQuat2Posso3`, `Posso32PosQuat`: Convert between position-quaternion and position-so(3) representations.
- `PosQuat2SE3`, `SE32PosQuat`: Convert between position-quaternion and SE(3) matrix representations.
- `RotPos2SE3`, `SE32RotPos`: Convert between rotation-position pairs and SE(3) matrices.
- `R6Pose2SE3`, `SE32R6Pose`: Convert between 6D pose and SE(3) matrix representations.

### Pose Preprocessing

- `PosQuatOutlierRemoval`: Remove outliers from pose data using a buffer.

### Robot Controller Functions

- `ErrorThreshold`: Check if the error is within a threshold.
- `SCurve`: Generate an S-curve for smooth command transitions.
- `KpPosso3`: Compute a proportional control command.
- `KpIBVSCircle2`: Compute a control command for image-based visual servoing.

