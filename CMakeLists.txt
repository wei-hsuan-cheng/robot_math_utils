cmake_minimum_required(VERSION 3.8)
project(robot_math_utils)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(ROBOT_MATH_UTILS_TEST_SOURCES src/robot_math_utils_test.cpp)

# Add executable
add_executable(robot_math_utils_test ${ROBOT_MATH_UTILS_TEST_SOURCES})

# Link libraries
ament_target_dependencies(robot_math_utils_test
  rclcpp
  Eigen3
)

# Install targets
install(TARGETS
  robot_math_utils_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()