# Robot Math Utils

A C++ math library for robotics applications, providing a collection of utility functions for robot kinematics, transformations, and control. This library is inspired by the Modern Robotics book codebase and includes additional functionalities for data logging, numerical computations, and robot controller functions.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
  - [Including the Library](#including-the-library)
  - [Example](#example)
- [Functions Overview](#functions-overview)
  - [Data Logging](#data-logging)
  - [Numerical Conditions](#numerical-conditions)
  - [Basic Math Functions](#basic-math-functions)
  - [Robot Transformation Functions](#robot-transformation-functions)
  - [SO(3) and so(3) Functions](#so3-and-so3-functions)
  - [SE(3) and se(3) Functions](#se3-and-se3-functions)
  - [Pose Preprocessing](#pose-preprocessing)
  - [Robot Controller Functions](#robot-controller-functions)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)
- [Contact](#contact)

## Features

- **Data Logging**: Functions to initialize and log data to CSV files.
- **Numerical Computations**: Utilities for near-zero checks, moving averages, and statistical calculations.
- **Math Utilities**: Basic mathematical functions, including error percentage, normalization, and random number generation.
- **Robot Transformations**: Functions for homogeneous coordinates, rotations, quaternions, and pose transformations.
- **Kinematics**: Implementations of exponential and logarithmic mappings for rotations and transformations.
- **Control Functions**: Utilities for error thresholds, S-curve generation, and proportional control.
- **Eigen Library Integration**: Built on top of the Eigen library for linear algebra operations.

## Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/your_username/robot_math_utils.git
