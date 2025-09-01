#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_14.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

using RM = RMUtils;

int main(int argc, char** argv) {
    // ROS 2 init
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_ik");

    std::cout << "\n----- Test numerical inverse kinematics (IK) with PoE screws -----\n";

    // === Robot model (TM5-700 example, same as test_fk) ===
    std::vector<DHParams> joints = {
        DHParams(Vector4d(  0,              0,            145.2e-3,     0        )),
        DHParams(Vector4d(-M_PI/2,         0,            0,           -M_PI/2  )),
        DHParams(Vector4d(  0,            329.0e-3,      0,            0        )),
        DHParams(Vector4d(  0,            311.5e-3,    -122.3e-3,     -M_PI/2  )),
        DHParams(Vector4d(-M_PI/2,         0,            106e-3,       0        )),
        DHParams(Vector4d(-M_PI/2,         0,            113.15e-3,    M_PI     ))
    };
    DHTable dh_table(joints);
    const size_t n = joints.size();

    // Optional: show DH and screw list
    dh_table.PrintTable();
    ScrewList screws = RM::ScrewListFromDH(dh_table);
    screws.PrintList();

    // === Ground-truth joint angles (to generate a reachable pos_quat_target) ===
    VectorXd theta_gt(n);
    // theta_gt << 8.94727e-07, -0.0916584, 2.01211, -1.13506, 1.5708, -1.5708; // same as test_fk demo
    theta_gt << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001; // near singularity
    // theta_gt << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // exact singularity

    std::cout << "\n-- Ground-truth theta [rad] -->\n" << theta_gt.transpose() << "\n";

    // Target pose from FK (PoE)
    PosQuat pos_quat_target = RM::FKPoE(screws, theta_gt);
    Matrix4d T_target = RM::PosQuat2TMat(pos_quat_target);
    std::cout << "\n-- Target pos_quat_b_e [m, quat] (from FKPoE(theta_gt)) -->\n" << pos_quat_target.pos.x() << ", "
              << pos_quat_target.pos.y() << ", "
              << pos_quat_target.pos.z() << ", "
              << pos_quat_target.quat.w() << ", "
              << pos_quat_target.quat.x() << ", "
              << pos_quat_target.quat.y() << ", "
              << pos_quat_target.quat.z() << "\n";
    

    // === IK initial guess ===
    VectorXd theta_init_error(n);
    theta_init_error << M_PI/20, -M_PI/25, 0.0, M_PI/30, -M_PI/10, 0.0; // error in theta initial guess
    VectorXd theta_init = theta_gt + theta_init_error; // initial guess for IK
    std::cout << "\n-- Initial guess theta_init [rad] -->\n" << theta_init.transpose() << "\n";

    // === IK solve ===
    VectorXd theta_sol = theta_init;
    const double eomg = 1e-7;      // orientation tol (‖ω‖) [rad]
    const double ev   = 1e-7;      // position tol (‖v‖) [m]
    int cur_iter = 0; // current iteration (for debugging)
    const int    max_iter = 100;
    const double lambda   = 1e-2;  // DLS damping (set 0.0 to disable)
    const double step_clip = 0.0;  // set >0.0 (e.g., 0.2) to cap per-step |Δθ|
    const bool   wrap_pi   = true;

    auto t_start = std::chrono::high_resolution_clock::now();
    bool ok = RM::IKNum(screws, pos_quat_target, theta_sol, cur_iter, eomg, ev, max_iter, lambda, step_clip, wrap_pi);
    auto t_end   = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = t_end - t_start;

    std::cout << "\n-- IK success -->\n" << (ok ? "[SUCCEEDED]" : "[FAILED]") << "\n";
    std::cout << "-- theta_sol [rad] -->\n" << theta_sol.transpose() << "\n";
    std::cout << "-- IK computation iteration/time [idx, ms] -->\n" << cur_iter << ", " << elapsed_ms.count() << std::endl;

    // === Verify: FK on solution and errors ===
    PosQuat pos_quat_fk = RM::FKPoE(screws, theta_sol);
    std::cout << "\n-- FK resultant pos_quat_b_e [m, quat] (from IK solution, FKPoE(theta_sol)) -->\n" << pos_quat_fk.pos.x() << ", "
              << pos_quat_fk.pos.y() << ", "
              << pos_quat_fk.pos.z() << ", "
              << pos_quat_fk.quat.w() << ", "
              << pos_quat_fk.quat.x() << ", "
              << pos_quat_fk.quat.y() << ", "
              << pos_quat_fk.quat.z() << "\n";
    
    std::cout << "\n-- FK resultant r6_pose [m, rad] (from IK solution, FKPoE(theta_sol)) -->\n"
              << RM::PosQuat2R6Pose(pos_quat_fk).transpose() << "\n";

    // Body/end-effector twist error: V_e = se3Vec( log( T_sol^{-1} * T_target ) ) = [v; w]
    Matrix4d T_sol = RM::PosQuat2TMat(pos_quat_fk);
    Matrix4d T_err = RM::Inv(T_sol) * T_target;
    Vector6d V_e = RM::se3Mat2R6Vec( RM::MatrixLog6(T_err) );
    Vector3d v_lin = V_e.head<3>();
    Vector3d w_ang = V_e.tail<3>();
    std::cout << "\n-- Residual body twist V_e = [v; w]^T -->\n" << V_e.transpose() << "\n";
    std::cout << "   ‖v‖ = " << v_lin.norm() << " [m],  ‖w‖ = " << w_ang.norm() << " [rad]\n";

    // // Joint error (wrapped to (-pi, pi])
    // VectorXd theta_err = theta_sol - theta_gt;
    // for (int i = 0; i < theta_err.size(); ++i) theta_err(i) = RM::ConstrainedAngle(theta_err(i), true);
    // std::cout << "\n-- Joint error (theta_sol - theta_gt, wrapped) [rad] -->\n" << theta_err.transpose() << "\n";

    // Joint error
    VectorXd theta_err = theta_sol - theta_gt;
    std::cout << "\n-- Joint error (theta_sol - theta_gt) [rad] -->\n" << theta_err.transpose() << "\n";


    // === Manipulability / Singularity check (no normalization) ===
    {
        MatrixXd J_e = RM::Jacob(screws, theta_gt);
        const double w         = RM::ManipulabilityIndex(J_e);
        const double log10w    = RM::ManipulabilityIndexLog10(J_e);
        const double sigma_min = RM::MinSingularValue(J_e);
        const double sigma_min_threshold = pow(10, -5);
        const bool   near_sing = RM::NearSingular(J_e, sigma_min_threshold); // tweak threshold as needed

        std::cout << "\n----- Manipulability / Singularity check (no normalization) -----\n";
        std::cout << "   w          = " << w << "   (log10(w) = " << log10w << ")\n";
        std::cout << "   sigma_min  = " << sigma_min << "\n";
        std::cout << "   nearSing? (thresh = " + std::to_string(sigma_min_threshold) + ")  = " << (near_sing ? "YES" : "NO") << "\n";
        if (near_sing) {
            std::cout << "   [WARN] Jacobian near singular; DLS (lambda=" << lambda
                    << ") may be critical here.\n";
        }
    }


    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
