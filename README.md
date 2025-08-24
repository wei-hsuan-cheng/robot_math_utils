# Robot Math Utils

A C++ Eigen-based maths library for robotics applications, providing a collection of utility functions for robot kinematics, transformations, and control. This library is inspired by the Modern Robotics book codebase and includes additional functionalities for data logging, numerical computations, and robot controller functions.

## Table of Contents

- [Robot Math Utils](#robot-math-utils)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
    - [Under development](#under-development)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
      - [Dependencies from `CMakeLists.txt`](#dependencies-from-cmakeliststxt)
  - [Functions Overview](#functions-overview)
    - [Data Logging](#data-logging)
    - [Numerical Conditions](#numerical-conditions)
    - [Sliding Window Functions](#sliding-window-functions)
    - [Basic Maths Functions](#basic-maths-functions)
      - [Random Variables](#random-variables)
    - [Homogeneous Coordinates](#homogeneous-coordinates)
      - [Homogeneous Coordinates in Cartesian Space (3D)](#homogeneous-coordinates-in-cartesian-space-3d)
      - [HomogeneousImage Coordinates (2D)](#homogeneousimage-coordinates-2d)
    - [$SO(3)$ and $so(3)$ Functions](#so3-and-so3-functions)
      - [Exp and Log in $SO(3)$](#exp-and-log-in-so3)
      - [$SO(3)$ Rotation Conversions](#so3-rotation-conversions)
      - [$SO(3)$ Rotation Transformations](#so3-rotation-transformations)
    - [$SE(3)$ and $se(3)$ Functions](#se3-and-se3-functions)
      - [Exp and Log in $SE(3)$](#exp-and-log-in-se3)
      - [$SE(3)$ Pose Conversions](#se3-pose-conversions)
      - [$SE(3)$ Pose Transformations](#se3-pose-transformations)
    - [Robot Kinematics, Motion Planning and Control](#robot-kinematics-motion-planning-and-control)
      - [Velocity Kinematics (Velocity Adjoint Maps)](#velocity-kinematics-velocity-adjoint-maps)
      - [Pose Preprocessing](#pose-preprocessing)
      - [Motion Mapping](#motion-mapping)
      - [Motion Planning](#motion-planning)
      - [Robot Controller Functions](#robot-controller-functions)
  - [Acknowledgements](#acknowledgements)
  - [Contact](#contact)

## Features

- **Data Logging**: Functions to initialise and log data to CSV files.
- **Numerical Computations**: Utilities for near-zero checks, moving averages, and statistical calculations.
- **Maths Utilities**: Basic mathematical functions, including error percentage, normalisation, and random number generation.
- **Robot Transformations**: Functions for homogeneous coordinates, $SO(3)$ rotations, quaternions, and $SE(3)$ pose transformations.
- **Exp and Log Maps**: Implementations of exponential and logarithmic mappings for $SO(3)$ rotations and $SE(3)$ transformations.
- **Motion Mapping**: Map from one workspace/velocity into another. Commonly used in telerobotics.
- **Motion Planning**: Screw motions and S-curve generation, etc.
- **Controller Functions**: Utilities for error thresholds and kinematic controller.
- **Eigen Library Integration**: Built on top of the Eigen library for linear algebra operations.
  
### Under development

- **Kinematics**: velocity/differential kinemtaics (Jacobian) and numerical inverse kinematics.

## Installation

This repository is ROS 2-based and utilises several dependencies that are common in robotics applications. Follow the instructions below to build the package in a ROS 2 environment.

### Prerequisites

Make sure you have the following installed:

- **ROS 2 humble**: Installed and properly sourced. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) if you haven't set it up yet.
- **C++17 Compiler**: Ensure your compiler supports C++17.
- **Dependencies**: The package depends on several libraries and ROS 2 packages.

#### Dependencies from `CMakeLists.txt`

- `Eigen3`
- ROS 2 packages:
  - `rclcpp`

Clone this repo into your ROS 2 workspace, install dependencies, and build pkg:

```bash
cd ~/ros2_ws/src && git clone https://github.com/wei-hsuan-cheng/robot_math_utils.git

cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y

cd ~/ros2_ws && colcon build --packages-select robot_math_utils && . install/setup.bash
```

Run the ROS 2 node for test:

```bash
cd ~/ros2_ws && . install/setup.bash
ros2 run robot_math_utils robot_math_utils_test
ros2 run robot_math_utils transform_time_analysis
ros2 run robot_math_utils test_fk
ros2 run robot_math_utils test_ik
```

## Functions Overview

### Data Logging

- `PrintVec`: Print an Eigen vector with a name.
- `InitDatalog`: Initialise a CSV file for data logging.
- `Datalog`: Log data vectors to a CSV file with timestamp and index.
- `SaveMat`: Log matrix to a CSV file.

### Numerical Conditions

- `NearZero`: Check if a value is near zero within a threshold.
- `ConstrainedAngle`: Normalise angle to a specified interval $(-\pi, \pi]$.
  
### Sliding Window Functions

- `MeanBuffer`: Compute the mean of vectors in a buffer.
- `StdBuffer`: Compute the standard deviation of vectors in a buffer.
- `MAvg`: Compute the moving average of vectors.

### Basic Maths Functions

- `ErrorPercentage`: Calculate the error percentage between measurement and ground truth.
- `Sinc`: Compute the sinc function.
- `ArcCos`: Compute the arccos function.
- `Norm`: Compute the norm of a vector.
- `MatNorm`: Compute the norm of a matrix.
- `Normalized`: Normalise a vector.
- `Transpose`: Compute the transpose of a matrix.
- `Tr`: Compute the trace of a matrix.
- `Det`: Compute the determinant of a matrix.
- `Inv`: Compute the inverse of a matrix.
- `LeftPInv`: Compute the (left) Moore–Penrose inverse of a matrix.
  
#### Random Variables

- `RandNorDist`: Generate a random number from a normal distribution.
- `RandNorDistVec`: Generate a random vector from a multivariate normal distribution.

### Homogeneous Coordinates

#### Homogeneous Coordinates in Cartesian Space (3D)

- `R3Vec2Homo`: Convert a 3D vector to homogeneous coordinates.
- `Homo2R3Vec`: Convert homogeneous coordinates to a 3D vector.

#### HomogeneousImage Coordinates (2D)

- `ImgCoord2Homo`: Convert image coordinates to homogeneous form.
- `Homo2ImgCoord`: Convert homogeneous image coordinates back to image coordinates.

### $SO(3)$ and $so(3)$ Functions

- `Quatx`, `Quaty`, `Quatz`: Generate quaternions for rotations about x, y, z axes.
- `Rotx`, `Roty`, `Rotz`: Generate rotation matrices about the x, y, z axes.
- `Rotxyz`, `Rotzyx`: Generate rotation matrices from Euler angles.
- `InvQuat`: Compute the inverse of a quaternion.
  
#### Exp and Log in $SO(3)$

- `R3Vec2so3Mat`: Convert a 3D vector to an $so(3)$ matrix.
- `so3Mat2R3Vec`: Convert an $so(3)$ matrix back to a 3D vector.
- `AxisAng3`: Convert a $so(3)$ vector to axis-angle representation.
- `so32Quat`: Convert a $so(3)$ vector to a quaternion.
- `Quat2so3`: Convert a quaternion to a $so(3)$ vector.
- `MatrixExp3`: Compute the exp map for screws in $so(3)$.
- `MatrixLog3`: Compute the log map for rotation matrices in $SO(3)$.

#### $SO(3)$ Rotation Conversions

- `Quat2zyxEuler`, `zyxEuler2Quat`: Conversion between quaternions and ZYX Euler angles.
- `Rot2zyxEuler`: Convert a rotation matrix to ZYX Euler angles.

#### $SO(3)$ Rotation Transformations

- `TransformQuats`: Apply a sequence of quaternion transformations.
- `TransformRots`: Apply a sequence of rotation matrix transformations.

### $SE(3)$ and $se(3)$ Functions

- `InvPosQuat`: Compute the inverse of a position-quaternion pair.
- `InvR6Pose`: Compute the inverse of a 6D pose.
  
#### Exp and Log in $SE(3)$

- `R6Vec2se3Mat`: Convert a 6D vector to an $se(3)$ matrix.
- `se3Mat2R6Vec`: Convert an $se(3)$ matrix back to a 6D vector.
- `AxisAng6`: Convert an $se(3)$ vector to axis-angle representation.
- `MatrixExp6`: Compute the exp map for screws in $se(3)$.
- `MatrixLog6`: Compute the log map for homogeneous transformation matrices in $SE(3)$.

#### $SE(3)$ Pose Conversions

- `PosQuat2R6Pose`, `R6Pose2PosQuat`: Convert between position-quaternion pairs and 6D poses.
- `PosQuat2Posso3`, `Posso32PosQuat`: Convert between position-quaternion pairs and position- $so(3)$ pairs.
- `PosQuat2TMat`, `TMat2PosQuat`: Convert between position-quaternion pairs and homogeneous transformation matrices.
- `PosRot2TMat`, `TMat2PosRot`: Convert between position-rotation matrix pairs and homogeneous transformation matrices.
- `R6Pose2TMat`, `TMat2R6Pose`: Convert between 6D poses and homogeneous transformation matrices

#### $SE(3)$ Pose Transformations

- `TransformPosQuats`: Apply a sequence of position-quaternion pair transformations.
- `TransformTMats`: Apply a sequence of transformations using homogeneous transformation matrices.
- `TransformR6Poses`: Apply a sequence of 6D pose transformations.


### Robot Kinematics, Motion Planning and Control

#### Velocity Transformation (Velocity Adjoint Maps)

- `Adjoint`: The adjoint map matrix.
- `AdjointE2B`: The adjoint map that transfer twist (spatial velocity) from end-effector representation into base representation.
- `AdjointB2E`: The adjoint map that transfer twist from base representation into end-effector representation.

#### Pose Preprocessing

- `PosQuatOutlierRemoval`: Remove outliers from pose data using a buffer.

#### Motion Mapping

- `AxisDecoupling`: Decouple the joystick axis motin so that each time only single-axis is triggered.
- `VelMapping`: Map from joystick positional signal to robot velocity/twist command.

#### Motion Planning

- `SCurve`: Smooth the velocity profile.
- `ScrewMotionPath`: Screw motion path generation (only waypoints).
- `ScrewMotionTraj`: Screw motion trajectory generation (waypoints and timestamps).

#### Robot Controller Functions

- `ErrorThreshold`: Check if the error is within a threshold.
- `KpPosso3`: Cartesian kinematic control for position-based visual servoing (PBVS).

## Acknowledgements

- **Modern Robotics**: Some functions are adapted from the [Modern Robotics book codebase](https://github.com/Le0nX/ModernRoboticsCpp).
- **Eigen Library**: This library heavily relies on the Eigen library for linear algebra operations.

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)


