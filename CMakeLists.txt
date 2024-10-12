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

# Suppress warnings for Eigen
add_compile_options(-Wno-class-memacces)

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
set(TRANSFORM_TIME_ANAYLSIS_SOURCES src/transform_time_analysis.cpp)
set(TEST_FK_SOURCES src/test_fk.cpp)

# Add executable
add_executable(robot_math_utils_test ${ROBOT_MATH_UTILS_TEST_SOURCES})
add_executable(transform_time_analysis ${TRANSFORM_TIME_ANAYLSIS_SOURCES})
add_executable(test_fk ${TEST_FK_SOURCES})

# Link libraries
ament_target_dependencies(robot_math_utils_test
  rclcpp
  Eigen3
)

ament_target_dependencies(transform_time_analysis
  rclcpp
  Eigen3
)

ament_target_dependencies(test_fk
  rclcpp
  Eigen3
)

# Install targets
install(TARGETS
  robot_math_utils_test
  transform_time_analysis
  test_fk
  
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