#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_5.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <random>
#include <vector>

using RM = RMUtils;

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    std::string node_name = "test_fk";
    auto node = rclcpp::Node::make_shared(node_name);
    std::cout << "\n----- Test forward kinematics (FK)  -----\n" << std::endl;

    const std::string& datalog_path = "./datalog/" + node_name;

    // DH parameters
    DHParams dh_j1 = DHParams(Vector4d(0, 0, 145.2 * pow(10, -3), 0));
    DHParams dh_j2 = DHParams(Vector4d(-M_PI / 2, 0, 0, -M_PI / 2));
    DHParams dh_j3 = DHParams(Vector4d(0, 329.0 * pow(10, -3), 0, 0));
    DHParams dh_j4 = DHParams(Vector4d(0, 311.5 * pow(10, -3), -122.3 * pow(10, -3), -M_PI / 2));
    DHParams dh_j5 = DHParams(Vector4d(-M_PI / 2, 0, 106 * pow(10, -3), 0));
    DHParams dh_j6 = DHParams(Vector4d(-M_PI / 2, 0, 113.15 * pow(10, -3), M_PI));

    // DH table
    DHTable dh_table({dh_j1, dh_j2, dh_j3, dh_j4, dh_j5, dh_j6});
    
    // Get the D-H parameters for each joint
    std::cout << "\n----- Get D-H parameters for each joint -----" << std::endl;
    for (int i = 1; i <= 6; ++i) {
        DHParams dh_ji = dh_table.GetJoint(i);
        std::cout << "dh_j" << i << ": " << dh_ji.alpha << ", " << dh_ji.a << ", " << dh_ji.d << ", " << dh_ji.theta << std::endl;
    }

    // Print the D-H parameters for each joint
    std::cout << "\n----- Print D-H parameters for each joint -----" << std::endl;
    for (int i = 1; i <= 6; ++i) {
        dh_table.PrintJoint(i);
    }

    // Print the entire D-H table
    std::cout << "\n----- D-H table -----" << std::endl;
    std::cout << dh_table << std::endl;

    

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
