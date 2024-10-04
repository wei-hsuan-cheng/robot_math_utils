#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_1.hpp"

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
    auto node = rclcpp::Node::make_shared("test_robot_math_utils");
    std::cout << "\n----- Starting RMUtils tests -----\n" << std::endl;

    double r2d = 180.0 / M_PI;
    double d2r = M_PI / 180.0;

    std::cout << "Hello, world!" << std::endl;

    // // Test Euler angles (from quat to rot and convert back to zyx_euler)
    // std::cout << "\n----- Test Euler angles -----\n" << std::endl;
    // Vector3d zyx_euler(30, 15, 45);
    // Matrix3d Rotzyx = RM::Rotzyx(zyx_euler * d2r);
    // Vector4d quat = RM::zyxEuler2Quat(zyx_euler * d2r);

    // Vector3d zyx_euler_from_Rot = RM::Rot2zyxEuler(Rotzyx) * r2d;
    // Vector3d zyx_euler_from_quat = RM::Quat2zyxEuler(quat) * r2d;
    // std::cout << "zyx_euler [deg] = [" << zyx_euler.transpose() << "]" << std::endl;
    // std::cout << "zyx_euler_from_Rot [deg] = [" << zyx_euler_from_Rot.transpose() << "]" << std::endl;
    // std::cout << "zyx_euler_from_quat [deg] = [" << zyx_euler_from_quat.transpose() << "]" << std::endl;

    // Test inverse quaternion
     std::cout << "\n----- Test inverse quaternion -----\n" << std::endl;
    Vector3d zyx_euler(30, 15, 45);
    Vector4d quat = RM::zyxEuler2Quat(zyx_euler * d2r) * 2.5;
    Vector4d quat_conj = RM::ConjQuat(quat);
    Vector4d quat_inv = RM::InvQuat(quat);
    Vector4d quat_quat_conj = RM::QuatMul(quat, RM::ConjQuat(quat));
    Vector4d quat_quat_inv = RM::QuatMul(quat, RM::InvQuat(quat));


    std::cout << "quat = [" << quat.transpose() << "]" << std::endl;
    std::cout << "quat_conj = [" << quat_conj.transpose() << "]" << std::endl;
    std::cout << "quat_inv = [" << quat_inv.transpose() << "]" << std::endl;
    std::cout << "quat_quat_conj = [" << quat_quat_conj.transpose() << "]" << std::endl;
    std::cout << "quat_quat_inv = [" << quat_quat_inv.transpose() << "]" << std::endl;






    // // Test transform multiple pos_quats
    // std::cout << "\n----- Test transform multiple pos_quats -----\n" << std::endl;
    // VectorXd pos_quat_1_2(7), pos_quat_2_3(7), pos_quat_3_4(7);
    // VectorXd temp_vec1(6);
    // temp_vec1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
    // pos_quat_1_2 = RM::R6Pose2PosQuat(temp_vec1);

    // VectorXd temp_vec2(6);
    // temp_vec2 << 0.0, 0.0, 0.0, 0.0, 0.6, 0.0;
    // pos_quat_2_3 = RM::R6Pose2PosQuat(temp_vec2);

    // VectorXd temp_vec3(6);
    // temp_vec3 << 0.0, 0.0, 0.0, 0.7, 0.0, 0.0;
    // pos_quat_3_4 = RM::R6Pose2PosQuat(temp_vec3);

    // VectorXd result_pos_quat = RM::TransformPosQuats({pos_quat_1_2, pos_quat_2_3, pos_quat_3_4});
    // VectorXd result_pose = RM::PosQuat2R6Pose(result_pos_quat);
    
    // RCLCPP_INFO(node->get_logger(), "\npose_1_4 [%f, %f, %f, %f, %f, %f]\n", 
    //             result_pose(0), result_pose(1), result_pose(2),
    //             result_pose(3), result_pose(4), result_pose(5));
    
    // // Test InvPosQuat
    // std::cout << "\n----- Test InvPosQuat -----\n" << std::endl;
    // VectorXd pos_quat_2_1 = RM::InvPosQuat(pos_quat_1_2);
    // VectorXd pose_2_1 = RM::PosQuat2R6Pose(pos_quat_2_1);
    // std::cout << "\npose_2_1 = [" << pose_2_1.transpose() << "]" << std::endl;

    // // Test InvR6Pose
    // std::cout << "\n----- Test InvR6Pose -----\n" << std::endl;
    // VectorXd pose_2_1 = RM::InvR6Pose(temp_vec1);
    // std::cout << "\npose_2_1 = [" << pose_2_1.transpose() << "]" << std::endl;



    // // Test matrix exponential and logarithm, and adjoint map
    // Vector3d uhat_theta(1.0, 2.0, 3.0);
    // Matrix3d so3Mat = RM::R3Vec2so3Mat(uhat_theta);
    // // std::cout << "so3Mat =\n" << so3Mat << std::endl;

    // Matrix3d R = RM::MatrixExp3(so3Mat);
    // // std::cout << "R =\n" << R << std::endl;

    // Matrix3d so3Mat_recovered = RM::MatrixLog3(R);
    // // std::cout << "so3Mat_recovered =\n" << so3Mat_recovered << std::endl;

    // Vector3d uhat_theta_recovered = RM::so3Mat2R3Vec(RM::MatrixLog3(R));
    // // std::cout << "uhat_theta_recovered [deg] = [" << uhat_theta_recovered * r2d << "]" << std::endl;

    // MatrixXd adj = RM::Adjoint(so3Mat, uhat_theta);
    // std::cout << "Adjoint matrix:\n" << adj << std::endl;


    // // Test robot controller functions
    // VectorXd pos_so3_m_cmd(6);
    // pos_so3_m_cmd << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3; 

    // // Example kp_pos_so3 matrix (3x3 identity)
    // MatrixXd kp_pos_so3 = MatrixXd::Identity(6, 6); 
    // bool target_reached = false; // Example flag

    // // Call the function and get both outputs
    // VectorXd twist_cmd = RM::KpPosso3(pos_so3_m_cmd, kp_pos_so3, target_reached);

    // // Print the results
    // std::cout << "Twist Command:\n" << twist_cmd << std::endl;


    // // Define a sample SE(3) transformation matrix T_1_2
    // MatrixXd T_1_2(4, 4);
    // T_1_2 << MatrixXd::Identity(3, 3), MatrixXd::Zero(3, 1),
    //          MatrixXd::Zero(1, 3), MatrixXd::Identity(1, 1);

    // // Call the SE32Posso3 function to compute the position and orientation error
    // VectorXd pos_so3_1_2 = RM::SE32Posso3(T_1_2);

    // // Print the result
    // RCLCPP_INFO(node->get_logger(), "Position and orientation error (pso3_1_2):\n[%f, %f, %f, %f, %f, %f]",
    //             pos_so3_1_2(0), pos_so3_1_2(1), pos_so3_1_2(2),
    //             pos_so3_1_2(3), pos_so3_1_2(4), pos_so3_1_2(5));
    

    // // Define some sample position and quaternion vectors
    // VectorXd pos_quat_1_2(7), pos_quat_2_3(7), pos_quat_3_4(7);
    // pos_quat_1_2 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // Translation (0.1, 0, 0), Identity quaternion
    // pos_quat_2_3 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // Translation (0, 0.2, 0), 60-degree rotation about X-axis
    // pos_quat_3_4 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // Translation (0, 0, 0.3), 60-degree rotation about Y-axis
    // VectorXd result_pos_quat = RM::TransformPosQuats({pos_quat_1_2, pos_quat_2_3, pos_quat_3_4});

    // // Print the result
    // RCLCPP_INFO(node->get_logger(), "pos_quat_1_4 =\n[%f, %f, %f, %f, %f, %f, %f]",
    //             result_pos_quat(0), result_pos_quat(1), result_pos_quat(2),
    //             result_pos_quat(3), result_pos_quat(4), result_pos_quat(5), result_pos_quat(6));


    // // Test so3Mat2R3Vec and R3Vec2so3Mat
    // Matrix3d so3Mat;
    // Vector3d so3;
    // so3Mat = RM::R3Vec2so3Mat(Vector3d(1.0, 2.0, 3.0));
    // so3 = RM::so3Mat2R3Vec(so3Mat);
    // RCLCPP_INFO(node->get_logger(), "so3Mat =\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
    //             so3Mat(0,0), so3Mat(0,1), so3Mat(0,2),
    //             so3Mat(1,0), so3Mat(1,1), so3Mat(1,2),
    //             so3Mat(2,0), so3Mat(2,1), so3Mat(2,2));


    // // Test Sinc function
    // float x = 0;
    // double sinc_x = RM::Sinc(x);
    // RCLCPP_INFO(node->get_logger(), "Sinc(%f) = %f", x, sinc_x);
    

    // // Test homogeneous coordinates
    // Vector3d v1(1.0, 2.0, 3.0), v2(4.0, 5.0, 6.0), v3(7.0, 8.0, 9.0);
    // double s = 0.5;
    // std::vector<Vector4d> v_hs = RM::R3Vecs2Homos({v1, v2, v3}, s);
    // std::vector<Vector3d> vs = RM::Homos2R3Vecs(v_hs);
    // for (const auto& v : v_hs) {
    //     std::cout << "Homogeneous vector: [" << v << "]" << std::endl;
    // }
    // for (const auto& v : vs) {
    //     std::cout << "Vector: [" << v << "]" << std::endl;
    // }




    // std::cout << "\n----- Starting SE(3) Tests -----\n" << std::endl;
    // // Test the R6Vec2se3Mat and se3Mat2R6Vec functions
    // VectorXd r6_vec(6);
    // r6_vec << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;  // Example SE(3) vector
    
    // // Convert SE(3) vector to matrix form
    // MatrixXd se3_mat = RM::R6Vec2se3Mat(r6_vec);
    // std::cout << "SE(3) matrix from vector:\n" << se3_mat << std::endl;

    // // Convert back from matrix to SE(3) vector
    // VectorXd r6_vec_recovered = RM::se3Mat2R6Vec(se3_mat);
    // std::cout << "Recovered SE(3) vector from matrix:\n" << r6_vec_recovered.transpose() << std::endl;

    // // Test the AxisAng6 function
    // VectorXd axis_ang6 = RM::AxisAng6(r6_vec);
    // std::cout << "Axis-Angle representation of SE(3):\n" << axis_ang6.transpose() << std::endl;

    // // Test the MatrixExp6 and MatrixLog6 functions
    // MatrixXd exp_se3 = RM::MatrixExp6(se3_mat);
    // std::cout << "Matrix exponential of SE(3):\n" << exp_se3 << std::endl;

    // MatrixXd log_se3 = RM::MatrixLog6(exp_se3);
    // std::cout << "Matrix logarithm of SE(3):\n" << log_se3 << std::endl;



    


    
    
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
