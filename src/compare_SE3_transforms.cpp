#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_2.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <random>
#include <vector>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using RM = RMUtils;

// Include the fixed-size types from RMUtils
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    std::string node_name = "compare_SE3_transforms";
    auto node = rclcpp::Node::make_shared(node_name);
    std::cout << "\n----- Starting comparing SE(3) transforms -----\n" << std::endl;

    const std::string& datalog_path = "./datalog/" + node_name;

    // Number of random pos_quats to generate
    int N = 10000; // You can change N to 100, 1000, 10000, etc.

    /* Generate random pos_quats (positions and quaternions) */
    std::vector<Vector7d> pos_quats; // Use Vector7d instead of Eigen::VectorXd
    std::vector<Matrix4d> SE3s;      // Use Matrix4d instead of Eigen::MatrixXd
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
        Vector7d pos_quat; // Use Vector7d
        pos_quat << position, quaternion;
        Matrix4d SE3 = RM::PosQuat2SE3(pos_quat);
        pos_quats.push_back(pos_quat);
        SE3s.push_back(SE3);
    }    

    // Measure time for position + quaternion method
    auto start = std::chrono::high_resolution_clock::now();
    Vector7d result_pos_quat = RM::TransformPosQuats(pos_quats);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_pos_quat = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Measure time for homogeneous matrix method
    start = std::chrono::high_resolution_clock::now();
    Matrix4d result_SE3 = RM::TransformSE3s(SE3s);
    end = std::chrono::high_resolution_clock::now();
    auto duration_SE3 = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Print results
    std::cout << "\n----- Results -----" << std::endl;
    std::cout << "Number of transformations: " << N << std::endl;
    std::cout << "result_pos_so3_from_quat = " << RM::PosQuat2Posso3(result_pos_quat).transpose() << std::endl;
    std::cout << "result_pos_so3_from_SE3 = " << RM::SE32Posso3(result_SE3).transpose() << std::endl;
    std::cout << "Difference between the two results [m, rad] = " << (RM::PosQuat2Posso3(result_pos_quat) - RM::SE32Posso3(result_SE3)).norm() << std::endl;
    // Output the timings 
    std::cout << "\n----- Time elapsed -----" << std::endl;
    std::cout << "PosQuat transform time (avg)  = " << static_cast<double>(duration_pos_quat) / static_cast<double>(N) << " [us]" << std::endl;
    std::cout << "SE3 matrix transform time (avg) = " << static_cast<double>(duration_SE3) / static_cast<double>(N) << " [us]" << std::endl;

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
