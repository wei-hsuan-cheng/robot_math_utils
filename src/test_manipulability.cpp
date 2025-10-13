#include <rclcpp/rclcpp.hpp>
#include "robot_kinematics_utils/robot_kinematics_utils_v1_0.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

using RM = RMUtils;
using RK = RKUtils;
using HighResClock = std::chrono::high_resolution_clock;

static inline double elapsedUs(const HighResClock::time_point& t0,
                               const HighResClock::time_point& t1) {
    return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

// Central difference for w(θ) via RK state updater
static double central_diff_w(RK& rk, const VectorXd& theta,
                             int j, double h)
{
    VectorXd tp = theta;
    VectorXd tm = theta;
    tp(j) += h;
    tm(j) -= h;
    VectorXd qd_zero = VectorXd::Zero(rk.dof());
    rk.UpdateRobotState(tp, qd_zero);
    double wp = rk.manipulability();
    rk.UpdateRobotState(tm, qd_zero);
    double wm = rk.manipulability();
    return (wp - wm) / (2.0 * h);
}

int main(int argc, char** argv) {
    // ROS 2 init
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_manipulability");

    std::cout << "\n----- Test manipulability gradient (PoE, body Jacobian) -----\n";

    // === Robot model (TM5-700 example, same as test_ik) ===
    std::vector<DHParams> joints = {
        DHParams(Vector4d(  0,              0,            145.2e-3,     0        )),
        DHParams(Vector4d(-M_PI/2,         0,            0,           -M_PI/2  )),
        DHParams(Vector4d(  0,            329.0e-3,      0,            0        )),
        DHParams(Vector4d(  0,            311.5e-3,    -122.3e-3,     -M_PI/2  )),
        DHParams(Vector4d(-M_PI/2,         0,            106e-3,       0        )),
        DHParams(Vector4d(-M_PI/2,         0,            113.15e-3,    M_PI     ))
    };
    DHTable dh_table(joints);
    const int n = static_cast<int>(joints.size());

    // Minimal metadata for consistent prints
    {
        const std::string robot_name = "tm5_700";
        const std::string base_frame = "base";
        const std::string ee_frame   = "flange";
        std::vector<std::string> jnames; jnames.reserve(n);
        for (int i = 0; i < n; ++i) jnames.emplace_back("joint_" + std::to_string(i + 1));
        MatrixXd jl = MatrixXd::Zero(n, 4);
        dh_table.setMeta(robot_name, base_frame, ee_frame, jnames, jl);
    }

    RK rk(dh_table);

    // print rk properties and screw lists
    std::cout << "Robot name: " << rk.screws().robot_name << "\n";
    std::cout << "  DOF: " << rk.dof() << "\n";
    rk.screws().PrintList();

    // Random test configurations (skip near singular ones)
    const int num_trials = 1000;
    int success_count = 0;
    const double h = 1e-8;              // finite-difference step
    const double near_sing_thresh = std::pow(10, -5);
    const double rel_tol = 1e-3;        // relative tolerance for gradient check
    const double abs_tol = 1e-6;        // absolute tolerance fallback

    // Random generator around zero
    VectorXd mean = VectorXd::Zero(n);
    MatrixXd cov  = (M_PI/8.0) * VectorXd::Ones(n).asDiagonal();

    // Accumulators for average runtime over num_trials
    double sum_us_gradJ = 0.0;
    double sum_us_rk_update = 0.0;
    int time_count = 0;

    for (int trial = 0; trial < num_trials; ++trial) {
        // Random small perturbation check: Δw ≈ dw_dq · Δθ
        // Current robot state
        VectorXd theta = RM::RandNorDistVec(mean, cov);
        VectorXd qd_zero = VectorXd::Zero(n);
        
        auto t0 = HighResClock::now();
        rk.UpdateRobotState(theta, qd_zero);
        auto t1 = HighResClock::now();
        
        double w = rk.manipulability(); // w(k) 
        VectorXd dw_dq = rk.manipulability_gradient(); // dw_dq(k)

        auto t2 = HighResClock::now();
        VectorXd dw_dq_from_J = RK::ManipulabilityGradient(rk.jacob()); // from J directly
        auto t3 = HighResClock::now();
        
        // Avoid singular ones
        if (RK::NearSingular(rk.jacob(), near_sing_thresh)) {
            --trial; // resample
            continue;
        }

        
        // Next time step's robot state
        VectorXd dtheta = RM::RandNorDistVec(mean, cov) * h; // tiny step Δθ
        rk.UpdateRobotState(theta + dtheta, qd_zero);
        
        double w_plus = rk.manipulability(); // w(k+1)
        double dw_num = w_plus - w;
        double dw_lin = dw_dq.transpose() * dtheta;
        double abs_err_dw = std::abs(dw_lin - dw_num);
        double rel_err_dw = abs_err_dw / std::max(1e-12, std::abs(dw_num));
        const bool pass_dw = (rel_err_dw < rel_tol) || (abs_err_dw < abs_tol);

        sum_us_rk_update += elapsedUs(t0, t1);
        sum_us_gradJ   += elapsedUs(t2, t3);
        ++time_count;

        if (pass_dw) {
            ++success_count;
        } else {
            std::cout << "  [WARNING] Finite-difference perturbation check failed!\n";
            std::cout << "  Δw (num)=" << dw_num << ",  dw_dq · Δθ=" << dw_lin
                      << ",  abs err=" << abs_err_dw
                      << ",  rel err=" << rel_err_dw << "\n";
        }
    }

    std::cout << "\n===== All tests done =====\n";
    std::cout << "Successful trials: " << success_count << "/" << num_trials << " ("
              << (static_cast<double>(success_count) / static_cast<double>(num_trials) * 100.0)
              << "%)\n";

    if (time_count > 0) {
        std::cout << "Average runtime over " << time_count << " trials:\n";
        std::cout << "  Time/Rate for RK state update: " << (sum_us_rk_update / time_count) << " [us] ("
                  << 1e6 / (sum_us_rk_update / time_count) << " [Hz])\n";
        std::cout << "  Time/Rate for ManipulabilityGradient(J):   " << (sum_us_gradJ / time_count) << " [us] ("
                  << 1e6 / (sum_us_gradJ / time_count) << " [Hz])\n";
    }

    rclcpp::shutdown();
    return 0;
}
