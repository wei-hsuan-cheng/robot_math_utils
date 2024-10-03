cmake_minimum_required(VERSION 3.8)
project(robot_math_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration visualization surface filters segmentation)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED) 
find_package(std_srvs REQUIRED)

# Dependencies list
set(dependencies
    ament_cmake
    geometry_msgs
    visualization_msgs
    pcl_conversions
    rcl_interfaces
    rclcpp
    rcutils
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_msgs
    PCL
    OpenCV
    SDL2
    Boost
    cv_bridge
    image_transport
    vision_msgs
    Eigen3 
    std_srvs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(ROBOT_MATH_UTILS_TEST_SOURCES src/robot_math_utils_test.cpp)

# Add executable
add_executable(robot_math_utils_test ${ROBOT_MATH_UTILS_TEST_SOURCES})

# Link libraries
target_link_libraries(robot_math_utils_test ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

# Add dependencies
ament_target_dependencies(robot_math_utils_test ${dependencies})

# Define installation rules
install(TARGETS
  robot_math_utils_test

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
