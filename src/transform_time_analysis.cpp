#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_4.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <random>
#include <vector>

using RM = RMUtils;
using std::int64_t;

std::tuple<Quaterniond, int64_t, double> TransformQuats(const std::vector<Quaterniond>& quats) {
    if (quats.empty()) {
        throw std::invalid_argument("The input list of quaternions is empty.");
    }
    Quaterniond quat_init = quats[0];
    int64_t total_duration_quat = 0;
    size_t num_iterations = quats.size() > 1 ? quats.size() - 1 : 1;  // To avoid division by zero
    for (size_t i = 1; i < quats.size(); ++i) {
        auto start_quat = std::chrono::high_resolution_clock::now();
        quat_init *= quats[i];
        auto end_quat = std::chrono::high_resolution_clock::now();

        int64_t duration_quat = std::chrono::duration_cast<std::chrono::nanoseconds>(end_quat - start_quat).count();
        total_duration_quat += duration_quat;
    }
    // Calculate average duration
    double avg_duration_quat = static_cast<double>(total_duration_quat) / num_iterations;
    return std::make_tuple(quat_init, total_duration_quat, avg_duration_quat);
}

std::tuple<Matrix3d, int64_t, double> TransformRots(const std::vector<Matrix3d>& Rs) {
    if (Rs.empty()) {
        throw std::invalid_argument("Input vector of rotation matrices is empty.");
    }
    Matrix3d result = Rs[0];
    int64_t total_duration_rot = 0;
    size_t num_iterations = Rs.size() > 1 ? Rs.size() - 1 : 1;  // To avoid division by zero

    for (size_t i = 1; i < Rs.size(); ++i) {
        auto start_rot = std::chrono::high_resolution_clock::now();
        result = result * Rs[i];
        auto end_rot = std::chrono::high_resolution_clock::now();
        int64_t duration_rot = std::chrono::duration_cast<std::chrono::nanoseconds>(end_rot - start_rot).count();
        total_duration_rot += duration_rot;
    }
    // Calculate average duration
    double avg_duration_rot = static_cast<double>(total_duration_rot) / num_iterations;
    return std::make_tuple(result, total_duration_rot, avg_duration_rot);
}

std::tuple<PosQuat, int64_t, double> TransformPosQuats(const std::vector<PosQuat>& pos_quats) {
    if (pos_quats.empty()) {
        throw std::invalid_argument("The input list of pos_quats is empty.");
    }
    PosQuat pos_quat_accum = pos_quats[0];
    int64_t total_duration_pos_quat = 0;
    size_t num_iterations = pos_quats.size() > 1 ? pos_quats.size() - 1 : 1;

    for (size_t i = 1; i < pos_quats.size(); ++i) {
        const PosQuat& pos_quat_i = pos_quats[i];

        auto start_pos_quat = std::chrono::high_resolution_clock::now();
        // Rotate and translate the position
        pos_quat_accum.pos = pos_quat_accum.quat * pos_quat_i.pos + pos_quat_accum.pos;
        // Compute the new quaternion
        pos_quat_accum.quat *= pos_quat_i.quat;
        auto end_pos_quat = std::chrono::high_resolution_clock::now();

        int64_t duration_pos_quat = std::chrono::duration_cast<std::chrono::nanoseconds>(end_pos_quat - start_pos_quat).count();
        total_duration_pos_quat += duration_pos_quat;
    }
    // Calculate average duration
    double avg_duration_pos_quat = static_cast<double>(total_duration_pos_quat) / num_iterations;
    return std::make_tuple(pos_quat_accum, total_duration_pos_quat, avg_duration_pos_quat);
}

std::tuple<Matrix4d, int64_t, double> TransformTMats(const std::vector<Matrix4d>& TMats) {
    if (TMats.empty()) {
        throw std::invalid_argument("Input vector of TMat matrices is empty.");
    }
    Matrix4d result = TMats[0];
    int64_t total_duration_TMat = 0;
    size_t num_iterations = TMats.size() > 1 ? TMats.size() - 1 : 1;

    for (size_t i = 1; i < TMats.size(); ++i) {
        auto start_TMat = std::chrono::high_resolution_clock::now();
        result = result * TMats[i];
        auto end_TMat = std::chrono::high_resolution_clock::now();
        int64_t duration_TMat = std::chrono::duration_cast<std::chrono::nanoseconds>(end_TMat - start_TMat).count();
        total_duration_TMat += duration_TMat;
    }
    // Calculate average duration
    double avg_duration_TMat = static_cast<double>(total_duration_TMat) / num_iterations;
    return std::make_tuple(result, total_duration_TMat, avg_duration_TMat);
}

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    std::string node_name = "transform_time_analysis";
    auto node = rclcpp::Node::make_shared(node_name);
    std::cout << "\n----- Comparing time elapsed in different transformation methods  -----\n" << std::endl;

    const std::string& datalog_path = "./datalog/" + node_name;

    // Number of random pos_quats to generate
    int N = 10000;

    /* Generate random pos_quats (positions and quaternions) */
    std::vector<Quaterniond> quats; 
    std::vector<Matrix3d> Rots;
    std::vector<PosQuat> pos_quats;
    std::vector<Matrix4d> TMats;      
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> pos_dist(-1.0, 1.0);   // Position between -1 and 1
    std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI); // Angle between -pi and pi

    for (int i = 0; i < N; ++i) {
        // Random position and axis-angle
        Vector3d position(pos_dist(rng), pos_dist(rng), pos_dist(rng));
        Vector3d axis(pos_dist(rng), pos_dist(rng), pos_dist(rng));
        if (axis.norm() == 0) {
            axis = Vector3d(1, 0, 0); // Avoid zero axis
        } else {
            axis.normalize();
        }
        double angle = angle_dist(rng);
        Vector3d so3 = axis * angle;
        
        Quaterniond quat = RM::so32Quat(so3);
        Matrix3d Rot = quat.toRotationMatrix();
        quats.push_back(quat);
        Rots.push_back(Rot);

        Vector6d pos_so3;
        pos_so3 << position, so3;
        
        PosQuat pos_quat(position, quat);
        pos_quats.push_back(pos_quat);

        Matrix4d TMat = RM::PosQuat2TMat(pos_quat);
        TMats.push_back(TMat);
    }    

    // Measure time for quaternion method
    auto [result_quat, total_duration_quat, avg_duration_quat] = TransformQuats(quats);
    // Measure time for rotation matrix method
    auto [result_rot, total_duration_rot, avg_duration_rot] = TransformRots(Rots);
    // Measure time for position + quaternion method
    auto [result_pos_quat, total_duration_pos_quat, avg_duration_pos_quat] = TransformPosQuats(pos_quats);
    // Measure time for homogeneous matrix method
    auto [result_TMat, total_duration_TMat, avg_duration_TMat] = TransformTMats(TMats);

    // Print results
    // std::cout << "\n----- Quat v.s. Rotation matrix -----" << std::endl;
    // std::cout << "Number of rotations: " << N << std::endl;
    // std::cout << "result_so3_from_quat = " << RM::Quat2so3(result_quat).transpose() << std::endl;
    // std::cout << "result_so3_from_rot = " << RM::Rot2so3(result_rot).transpose() << std::endl;
    // std::cout << "\n----- Time elapsed -----" << std::endl;
    // std::cout << "Quaternion transform time (avg) = " << avg_duration_quat << " [ns]" << std::endl;
    // std::cout << "Rotation matrix transform time (avg) = " << avg_duration_rot << " [ns]" << std::endl;

    std::cout << "\n----- PosQuat v.s. Transformation matrix -----" << std::endl;
    std::cout << "Number of transformations: " << N << std::endl;
    std::cout << "result_pos_so3_from_pos_quat = " << RM::PosQuat2Posso3(result_pos_quat).transpose() << std::endl;
    std::cout << "result_pos_so3_from_TMat = " << RM::TMat2Posso3(result_TMat).transpose() << std::endl;
    std::cout << "\n----- Time elapsed -----" << std::endl;
    std::cout << "PosQuat transform time (avg)  = " << avg_duration_pos_quat << " [ns]" << std::endl;
    std::cout << "TMat matrix transform time (avg) = " << avg_duration_TMat << " [ns]" << std::endl;

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
