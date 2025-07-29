#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_12.hpp"

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

    const std::string& datalog_path = "./datalog/" + node_name;

    std::cout << "\n----- Test forward kinematics (FK) by D-H parameters and product of exponentials (PoE) -----\n" << std::endl;

    // DH parameters for each joint (e.g. TM5-700 robotic arm)
    std::vector<DHParams> joints = {
        DHParams(Eigen::Vector4d(  0,              0,            145.2e-3,     0        )),
        DHParams(Eigen::Vector4d(-M_PI/2,         0,            0,           -M_PI/2  )),
        DHParams(Eigen::Vector4d(  0,            329.0e-3,      0,            0        )),
        DHParams(Eigen::Vector4d(  0,            311.5e-3,    -122.3e-3,     -M_PI/2  )),
        DHParams(Eigen::Vector4d(-M_PI/2,         0,            106e-3,       0        )),
        DHParams(Eigen::Vector4d(-M_PI/2,         0,            113.15e-3,    M_PI     ))
    };
    // DH table
    DHTable dh_table(joints);
    size_t n = joints.size();
    
    // // Get the D-H parameters for each joint
    // std::cout << "\n----- Get D-H parameters for each joint -----" << std::endl;
    // for (int i = 1; i <= 6; ++i) {
    //     DHParams dh_ji = dh_table.GetJoint(i);
    //     std::cout << "dh_j" << i << ": " << dh_ji.alpha << ", " << dh_ji.a << ", " << dh_ji.d << ", " << dh_ji.theta << std::endl;
    // }

    // // Print the D-H parameters for each joint
    // std::cout << "\n----- Print D-H parameters for each joint -----" << std::endl;
    // for (int i = 1; i <= 6; ++i) {
    //     dh_table.PrintJoint(i);
    // }


    // Print the entire D-H table
    std::cout << "\n----- D-H table -----" << std::endl;
    std::cout << dh_table << std::endl;


    // Generate ScrewList from DHTable
    ScrewList screw_list_ = RM::ScrewListFromDH(dh_table);
    screw_list_.PrintScrewList();

    // Test FK using the two methods
    VectorXd theta_list(n);

    // theta_list << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;  // zero configuration for testing
    theta_list << M_PI/18, -M_PI/9, M_PI/12, -M_PI/6, M_PI/4, -M_PI/2;  // sample joint offsets
    // theta_list << M_PI/2, 0.0, 0.0, 0.0, 0.0, 0.0;  // sample joint offsets
    // theta_list << M_PI/2, M_PI/4, 0.0, 0.0, 0.0, 0.0;  // sample joint offsets
    std::cout << "\n-- FK test with given theta list [rad] -->\n" << theta_list.transpose() << "\n";

    // Test FKDH
    Eigen::Matrix4d T_b_e_dh = RM::PosQuat2TMat( RM::FKDH(dh_table, theta_list) );
    std::cout << "\n-- FKDH result T_b_e -->\n" << T_b_e_dh << "\n";

    // Test FKPoE
    Eigen::Matrix4d T_b_e_poe = RM::PosQuat2TMat( RM::FKPoE(screw_list_, theta_list) );
    std::cout << "\n-- FKPoE result T_b_e -->\n" << T_b_e_poe << "\n\n";

    // FK error (set DH as ground truth)
    Vector6d pos_so3_error = RM::TMat2Posso3(T_b_e_poe.inverse() * T_b_e_dh);
    std::cout << "\n-- FK pos_so3 error [m, rad] -->\n" << pos_so3_error.transpose() << "\n\n";

    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
