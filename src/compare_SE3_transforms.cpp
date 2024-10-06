#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_1.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using RM = RMUtils;

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    std::string node_name = "compare_SE3_transforms";
    auto node = rclcpp::Node::make_shared(node_name);
    std::cout << "\n----- Starting comparing SE(3) transforms -----\n" << std::endl;

    const std::string& datalog_path = "./datalog/" + node_name;

    // Number of random pos_quats to generate
    int N = 1000; // You can change N to 100, 1000, 10000, etc.

    /* Generate random pos_quats (positions and quaternions) */
    std::vector<Eigen::VectorXd> pos_quats;
    std::vector<Eigen::MatrixXd> SE3s;
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> pos_dist(-1.0, 1.0);     // Position between -1 and 1
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI); // Angle between -pi and pi

    for (int i = 0; i < N; ++i) {
        // Random position and so3
        Vector3d position(pos_dist(rng), pos_dist(rng), pos_dist(rng));
        Vector3d axis(pos_dist(rng), pos_dist(rng), pos_dist(rng));
        if (axis.norm() == 0) {
            axis = Vector3d(1, 0, 0); // Avoid zero axis
        } else {
            axis.normalize();
        }
        double angle = angle_dist(rng);
        Vector3d so3 = axis * angle;
        // Convert so3 to quaternion
        Vector4d quaternion = RM::so32Quat(so3);
        // pos_quat
        Eigen::VectorXd pos_quat(7);
        pos_quat << position, quaternion;
        MatrixXd SE3 = RM::PosQuat2SE3(pos_quat);
        pos_quats.push_back(pos_quat);
        SE3s.push_back(SE3);
    }    

    // Measure time for position + quaternion method
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd result_pos_quat = RM::TransformPosQuats(pos_quats);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_pos_quat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Measure time for homogeneous matrix method
    start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix4d result_se3 = RM::TransformSE3s(SE3s);
    end = std::chrono::high_resolution_clock::now();
    auto duration_se3 = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Output the timings
    std::cout << "----- Results -----" << std::endl;
    std::cout << "Number of transformations: " << N << std::endl;
    std::cout << "result_so3_from_quat = " << RM::PosQuat2Posso3(result_pos_quat).transpose() << std::endl;
    std::cout << "result_so3_from_SE3 = " << RM::SE32Posso3(result_se3).transpose() << std::endl;
    std::cout << "Position + Quaternion method took: " << duration_pos_quat << " microseconds." << std::endl;
    std::cout << "Homogeneous Matrix method took: " << duration_se3 << " microseconds." << std::endl;

    // For verification, compare the final results
    // Convert result_se3 back to pos_quat
    Eigen::VectorXd result_pos_quat_se3 = RM::SE32PosQuat(result_se3);

    // Compute the difference between the two results
    double position_error = (result_pos_quat.head<3>() - result_pos_quat_se3.head<3>()).norm();
    double quaternion_error = (result_pos_quat.tail<4>() - result_pos_quat_se3.tail<4>()).norm();

    std::cout << "Position difference between methods: " << position_error << std::endl;
    std::cout << "Quaternion difference between methods: " << quaternion_error << std::endl;

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
