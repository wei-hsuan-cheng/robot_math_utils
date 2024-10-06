#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_2.hpp"

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
    std::string node_name = "robot_math_utils_test";
    auto node = rclcpp::Node::make_shared(node_name);
    std::cout << "\n----- Starting RMUtils tests -----\n" << std::endl;

    const std::string& datalog_path = "./datalog/" + node_name;

    // // Test ConstrainedAngle
    // std::cout << "\n----- Test ConstrainedAngle -----\n" << std::endl;
    // double angle = -M_PI * 1.00000000001 - 5 * 2 * M_PI;
    // double angle_constrained = RM::ConstrainedAngle(angle, true);
    // std::cout << "angle = " << angle << std::endl;
    // std::cout << "angle_constrained = " << angle_constrained << std::endl;

    // // Test ArcCos
    // std::cout << "\n----- Test ArcCos -----\n" << std::endl;
    // double cos_val = cos(M_PI * 1.1 + 2 * M_PI);
    // double theta_rec = RM::ArcCos(cos_val, false);
    // std::cout << "theta_rec [deg] = " << theta_rec << std::endl;
    // std::cout << "theta_rec [rad] = " << theta_rec * RM::d2r << std::endl;

    
    // // Test RandNorDistVec for generating N = 100 data points
    // std::cout << "\n----- Test RandNorDistVec (Generating 100 data points) -----\n" << std::endl;
    // int N = 5000;  // Number of data points
    // VectorXd mean(2), std_dev(2);
    // mean << 1.0, 5.0;
    // std_dev << 0.5, 0.1;
    // MatrixXd cov = std_dev.asDiagonal() * RM::Transpose(std_dev.asDiagonal());
    // // Matrix to store all random vectors
    // Eigen::MatrixXd rand_vecs(N, mean.size());
    // // Generate N random vectors and store in the matrix
    // for (int i = 0; i < N; ++i) {
    //     VectorXd rand_vec = RM::RandNorDistVec(mean, cov);
    //     rand_vecs.row(i) = rand_vec.transpose();
    // }
    // // Save the generated data points to CSV
    // std::string filename = datalog_path + "/rand_vecs_" + std::to_string(N) + ".csv";
    // RM::SaveMat(rand_vecs, filename);
    // std::cout << "Random vectors saved to " << filename << std::endl;


    // Test quat with z-axis pi rotation
    std::cout << "\n----- Test quat with z-axis pi rotation -----\n" << std::endl;
    Vector4d quat = RM::Quatz(M_PI);
    std::cout << "quat = [" << quat.transpose() << "]" << std::endl;

    // // Test Euler angles (from quat to rot and convert back to zyx_euler)
    // std::cout << "\n----- Test Euler angles -----\n" << std::endl;
    // Vector3d zyx_euler(30, 15, 45);
    // Matrix3d Rotzyx = RM::Rotzyx(zyx_euler * RM::d2r);
    // Vector4d quat = RM::zyxEuler2Quat(zyx_euler * RM::d2r);

    // Vector3d zyx_euler_from_Rot = RM::Rot2zyxEuler(Rotzyx) * RM::r2d;
    // Vector3d zyx_euler_from_quat = RM::Quat2zyxEuler(quat) * RM::r2d;
    // std::cout << "zyx_euler [deg] = [" << zyx_euler.transpose() << "]" << std::endl;
    // std::cout << "zyx_euler_from_Rot [deg] = [" << zyx_euler_from_Rot.transpose() << "]" << std::endl;
    // std::cout << "zyx_euler_from_quat [deg] = [" << zyx_euler_from_quat.transpose() << "]" << std::endl;

    // // Test inverse quaternion
    // std::cout << "\n----- Test inverse quaternion -----\n" << std::endl;
    // Vector3d zyx_euler(30, 15, 45);
    // Vector4d quat = RM::zyxEuler2Quat(zyx_euler * d2r) * 2.5;
    // Vector4d quat_conj = RM::ConjQuat(quat);
    // Vector4d quat_inv = RM::InvQuat(quat);
    // Vector4d quat_quat_conj = RM::QuatMul(quat, RM::ConjQuat(quat));
    // Vector4d quat_quat_inv = RM::QuatMul(quat, RM::InvQuat(quat));

    // std::cout << "quat = [" << quat.transpose() << "]" << std::endl;
    // std::cout << "quat_conj = [" << quat_conj.transpose() << "]" << std::endl;
    // std::cout << "quat_inv = [" << quat_inv.transpose() << "]" << std::endl;
    // std::cout << "quat_quat_conj = [" << quat_quat_conj.transpose() << "]" << std::endl;
    // std::cout << "quat_quat_inv = [" << quat_quat_inv.transpose() << "]" << std::endl;



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

    // // Test PosQuats2RelativePosQuat
    // std::cout << "\n----- Test PosQuats2RelativePosQuat -----\n" << std::endl;
    // VectorXd pos_quat_1_2(7), pos_quat_1_3(7), pos_quat_4_5(7), pos_quat_5_6(7);
    // VectorXd pose_1_2(6);
    // pose_1_2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
    // pos_quat_1_2 = RM::R6Pose2PosQuat(pose_1_2);

    // VectorXd pose_1_3(6);
    // pose_1_3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.8;
    // pos_quat_1_3 = RM::R6Pose2PosQuat(pose_1_3);

    // VectorXd pose_4_5(6);
    // pose_4_5 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // pos_quat_4_5 = RM::R6Pose2PosQuat(pose_4_5);

    // VectorXd pose_5_6(6);
    // pose_5_6 << 0.0, 0.0, 0.3, 0.0, 0.0, 0.0;
    // pos_quat_5_6 = RM::R6Pose2PosQuat(pose_5_6);

    // VectorXd pos_quat_2_3 = RM::PosQuats2RelativePosQuat(pos_quat_1_2, pos_quat_1_3);
    // VectorXd pose_2_3_from_pos_quat = RM::PosQuat2R6Pose(pos_quat_2_3);
    // std::cout << "\npose_2_3_from_pos_quat = [" << pose_2_3_from_pos_quat.transpose() << "]" << std::endl;

    // // Test R6Poses2RelativeR6Pose and TransformR6Poses
    // std::cout << "\n----- Test R6Poses2RelativeR6Pose and TransformR6Poses -----\n" << std::endl;
    // VectorXd pose_2_3 = RM::R6Poses2RelativeR6Pose(pose_1_2, pose_1_3);
    // std::cout << "\npose_2_3 = [" << pose_2_3.transpose() << "]" << std::endl;

    // VectorXd pose_4_6 = RM::TransformR6Poses({pose_4_5, pose_5_6});
    // std::cout << "\npose_4_6 = [" << pose_4_6.transpose() << "]" << std::endl;

    

    
    
    // // Test quaternion Pi
    // std::cout << "\n----- Test quaternion Pi -----\n" << std::endl;
    // Vector4d quat = RM::Quatz(M_PI);
    // std::cout << "quat = [" << quat.transpose() << "]" << std::endl;
    // Vector3d so3_from_quat = RM::Quat2so3(quat);
    // Vector4d axis_angle_from_quat = RM::AxisAng3(so3_from_quat);
    // std::cout << "so3_from_quat = [" << so3_from_quat.transpose() << "]" << std::endl;
    // std::cout << "axis_angle_from_quat = [" << axis_angle_from_quat.transpose() << "]" << std::endl;



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

    // // Test ScrewMotion
    // std::cout << "\n----- Test ScrewMotion -----\n" << std::endl;
    // VectorXd pos_quat_b_e(7);
    // pos_quat_b_e << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    // VectorXd pos_quat_e_e_cmd = RM::R6Pose2PosQuat( (VectorXd(6) << 1.0, 2.0, 3.0, 0.0, 0.0, 0.75).finished() );
    // int N = 10;
    // double T = 5.0;
    
    // // ScrewMotionTraj
    // auto[pos_quat_b_e_traj, t]  = RM::ScrewMotionTraj(pos_quat_b_e, pos_quat_e_e_cmd, N, T);
    // // Print trajectory
    // for (int i = 0; i < N; i++) {
    //     std::cout << "t(" << i << ") [s] = " << t[i] << ", ";
    //     RM::PrintVec(RM::PosQuat2R6Pose( pos_quat_b_e_traj[i] ), "pose_b_e_traj(" + std::to_string(i) + ")");
    // }


    
    
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
