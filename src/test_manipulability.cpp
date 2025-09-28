#include <rclcpp/rclcpp.hpp>
#include "robot_math_utils/robot_math_utils_v1_16.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

using RM = RMUtils;

// Finite-difference derivative for a scalar function of joints using central difference
static double central_diff_w(const ScrewList& screws, const Eigen::VectorXd& theta,
                             int j, double h)
{
    Eigen::VectorXd tp = theta;
    Eigen::VectorXd tm = theta;
    tp(j) += h;
    tm(j) -= h;
    Eigen::MatrixXd Jp = RM::Jacob(screws, tp);
    Eigen::MatrixXd Jm = RM::Jacob(screws, tm);
    double wp = RM::ManipulabilityIndex(Jp);
    double wm = RM::ManipulabilityIndex(Jm);
    return (wp - wm) / (2.0 * h);
}

int main(int argc, char** argv) {
    // ROS 2 init
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_manipulability");

    std::cout << "\n----- Test manipulability gradient (PoE, body Jacobian) -----\n";

    // === Robot model (TM5-700 example, same as test_ik) ===
    std::vector<DHParams> joints = {
        DHParams(Eigen::Vector4d(  0,              0,            145.2e-3,     0        )),
        DHParams(Eigen::Vector4d(-M_PI/2,         0,            0,           -M_PI/2  )),
        DHParams(Eigen::Vector4d(  0,            329.0e-3,      0,            0        )),
        DHParams(Eigen::Vector4d(  0,            311.5e-3,    -122.3e-3,     -M_PI/2  )),
        DHParams(Eigen::Vector4d(-M_PI/2,         0,            106e-3,       0        )),
        DHParams(Eigen::Vector4d(-M_PI/2,         0,            113.15e-3,    M_PI     ))
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
        Eigen::MatrixXd jl = Eigen::MatrixXd::Zero(n, 4);
        dh_table.setMeta(robot_name, base_frame, ee_frame, jnames, jl);
    }

    ScrewList screws = RM::ScrewListFromDH(dh_table);

    // Random test configurations (skip near singular ones)
    const int num_trials = 5;
    const double h = 1e-6;              // finite-difference step
    const double near_sing_thresh = std::pow(10, -5);
    const double rel_tol = 1e-3;        // relative tolerance for gradient check
    const double abs_tol = 1e-6;        // absolute tolerance fallback

    // Random generator around zero
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd cov  = (M_PI/8.0) * Eigen::VectorXd::Ones(n).asDiagonal();

    for (int trial = 0; trial < num_trials; ++trial) {
        // Sample a configuration and avoid singular ones
        Eigen::VectorXd theta = RM::RandNorDistVec(mean, cov);
        Eigen::MatrixXd J = RM::Jacob(screws, theta);
        if (RM::NearSingular(J, near_sing_thresh)) {
            --trial; // resample
            continue;
        }

        double w = RM::ManipulabilityIndex(J);
        Eigen::VectorXd grad = RM::ManipulabilityGradient(J);                 // from J directly
        Eigen::VectorXd grad_poe = RM::ManipulabilityGradient(screws, theta); // PoE overload

        std::cout << "\n[Trial " << (trial+1) << "] w = " << w << "\n";

        // Check consistency of the two implementations
        double impl_diff = (grad - grad_poe).norm();
        std::cout << "  impl diff ‖grad(J) - grad(screws,θ)‖ = " << impl_diff << "\n";

        // Per-joint finite-difference check
        double max_abs_err = 0.0;
        double max_rel_err = 0.0;
        for (int j = 0; j < n; ++j) {
            double num = central_diff_w(screws, theta, j, h);
            double ana = grad(j);
            double abs_err = std::abs(ana - num);
            double denom = std::max(1e-12, std::abs(num));
            double rel_err = abs_err / denom;
            max_abs_err = std::max(max_abs_err, abs_err);
            max_rel_err = std::max(max_rel_err, rel_err);
            std::cout << "    j=" << j+1
                      << ":  grad(j) = " << ana
                      << ",  num ≈ " << num
                      << ",  abs err = " << abs_err
                      << ",  rel err = " << rel_err << "\n";
        }

        const bool pass = (max_rel_err < rel_tol) || (max_abs_err < abs_tol);
        std::cout << "  Summary: max_abs_err=" << max_abs_err
                  << ", max_rel_err=" << max_rel_err
                  << "  => " << (pass ? "[PASS]" : "[FAIL]") << "\n";

        // Random small perturbation check: Δw ≈ grad · Δθ
        Eigen::VectorXd dtheta = RM::RandNorDistVec(mean, cov) * 1e-4; // tiny step
        Eigen::MatrixXd Jp = RM::Jacob(screws, theta + dtheta);
        double w_plus = RM::ManipulabilityIndex(Jp);
        double dw_num = w_plus - w;
        double dw_lin = grad.dot(dtheta);
        double abs_err_dw = std::abs(dw_lin - dw_num);
        double rel_err_dw = abs_err_dw / std::max(1e-12, std::abs(dw_num));
        std::cout << "  Δw (num)=" << dw_num << ",  grad·Δθ=" << dw_lin
                  << ",  abs err=" << abs_err_dw
                  << ",  rel err=" << rel_err_dw << "\n";
    }

    rclcpp::shutdown();
    return 0;
}
