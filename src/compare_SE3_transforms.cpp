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

    // Number of random poses to generate
    int N = 1000; // You can change N to 100, 1000, 10000, etc.

    // Generate random poses (positions and quaternions)
    std::vector<Eigen::VectorXd> poses;

    // Random number generators
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> pos_dist(-1.0, 1.0);     // Position between -1 and 1
    std::uniform_real_distribution<double> angle_dist(0.0, 2 * M_PI); // Angle between 0 and 2*pi

    for (int i = 0; i < N; ++i) {
        // Generate random position vector
        Vector3d position(pos_dist(rng), pos_dist(rng), pos_dist(rng));

        // Generate random rotation vector (axis-angle)
        Vector3d axis(pos_dist(rng), pos_dist(rng), pos_dist(rng));
        if (axis.norm() == 0) {
            axis = Vector3d(1, 0, 0); // Avoid zero axis
        } else {
            axis.normalize();
        }
        double angle = angle_dist(rng);
        Vector3d so3 = axis * angle;
        Vector4d quaternion = RM::so32Quat(so3);

        // Combine position and quaternion into a single vector
        Eigen::VectorXd pos_quat(7);
        pos_quat << position, quaternion;
        poses.push_back(pos_quat);
    }

    // Function to compose transformations using position + quaternion
    auto composePosQuats = [](const std::vector<Eigen::VectorXd>& poses) -> Eigen::VectorXd {
        Eigen::VectorXd result = poses[0];
        for (size_t i = 1; i < poses.size(); ++i) {
            result = RM::TransformPosQuat(result, poses[i]);
        }
        return result;
    };

    // Function to compose transformations using homogeneous matrices
    auto composeSE3Matrices = [](const std::vector<Eigen::VectorXd>& poses) -> Eigen::Matrix4d {
        Eigen::Matrix4d result = RM::PosQuat2SE3(poses[0]);
        for (size_t i = 1; i < poses.size(); ++i) {
            Eigen::Matrix4d T = RM::PosQuat2SE3(poses[i]);
            result = result * T; // Matrix multiplication
        }
        return result;
    };

    // Measure time for position + quaternion method
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd result_pos_quat = composePosQuats(poses);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_pos_quat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Measure time for homogeneous matrix method
    start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix4d result_se3 = composeSE3Matrices(poses);
    end = std::chrono::high_resolution_clock::now();
    auto duration_se3 = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Output the timings
    std::cout << "Number of transformations: " << N << std::endl;
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
