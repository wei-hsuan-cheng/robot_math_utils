#ifndef ROBOT_KINEMATICS_UTILS_HPP
#define ROBOT_KINEMATICS_UTILS_HPP

// Author: Wei-Hsuan Cheng, johnathancheng0125@gmail.com, https://github.com/wei-hsuan-cheng
// GitHub repo: https://github.com/wei-hsuan-cheng/robot_math_utils
// v1_0, last edit: 251010
//
// Version history:
//  - Migrate the kinematics-related functions from robot_math_utils_v1_16.hpp to this new file.

#include "robot_math_utils/robot_math_utils_v1_16.hpp"
#include <limits>

// Short alias for the math/PoE utility
using RM = RMUtils;

// RobotKinematics: object-oriented wrapper around RMUtils kinematics.
// Holds a robot model (DH table and/or PoE screw list) and exposes
// FK / Jacobian / IK and manipulability utilities as methods.
class RobotKinematics {
public:
    // Constructors
    explicit RobotKinematics(const DHTable& dh)
    : dh_table_(dh), screws_(RM::ScrewListFromDH(dh))
    {
        n_ = static_cast<int>(screws_.screw_list.cols());
    }

    explicit RobotKinematics(const ScrewList& screws)
    : screws_(screws)
    {
        n_ = static_cast<int>(screws_.screw_list.cols());
    }

    // Degrees of freedom
    int dof() const { return n_; }

    // Accessors
    const DHTable&    dh()     const { return dh_table_; }
    const ScrewList&  screws() const { return screws_; }

    // Forward Kinematics (DH) — reimplemented here
    PosQuat FKDH(const VectorXd& theta_list) const {
        // Number of joints
        size_t n = dh_table_.joints.size();
        if (static_cast<size_t>(theta_list.size()) != n) {
            throw std::invalid_argument("[RobotKinematics::FKDH] theta_list size mismatch");
        }
        // Compose transforms for each joint using D-H parameters
        std::vector<PosQuat> transforms;
        transforms.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const auto& dh = dh_table_.joints[i];
            double theta = dh.theta + theta_list(static_cast<int>(i));
            transforms.push_back(RM::DHTransform(dh.alpha, dh.a, dh.d, theta));
        }
        return RM::TransformPosQuats(transforms);
    }

    // Forward Kinematics (PoE, body/end-effector frame screws) — reimplemented here
    PosQuat FKPoE(const VectorXd& theta_list) const {
        const int n = static_cast<int>(theta_list.size());
        if (screws_.screw_list.cols() != n) {
            throw std::invalid_argument("[RobotKinematics::FKPoE] theta_list size mismatch");
        }
        // start from home pose M
        Eigen::Matrix4d T = RM::PosQuat2TMat(screws_.M);
        for (int i = 0; i < n; ++i) {
            Vector6d screw6 = screws_.screw_list.col(i) * theta_list(i);
            Eigen::Matrix4d exp_se3 = RM::MatrixExp6(RM::R6Vec2se3Mat(screw6));
            T = T * exp_se3;
        }
        return RM::TMat2PosQuat(T);
    }

    // Differential Kinematics (body Jacobian) — reimplemented
    MatrixXd Jacob(const VectorXd& theta_list) const {
        const int n = static_cast<int>(screws_.screw_list.cols());
        if (theta_list.size() != n) {
            throw std::invalid_argument("[RobotKinematics::Jacob] theta_list size mismatch");
        }
        MatrixXd J_e = screws_.screw_list;       // 6 x n
        Matrix4d T  = Matrix4d::Identity();      // accumulates Π exp(-S_{k} θ_{k}) from right
        for (int i = n - 2; i >= 0; --i) {
            T = T * RM::MatrixExp6( RM::R6Vec2se3Mat(-screws_.screw_list.col(i + 1) * theta_list(i + 1)) );
            auto TR = RM::TMat2PosRot(T);
            J_e.col(i) = RM::Adjoint(TR.rot, TR.pos) * screws_.screw_list.col(i);
        }
        return J_e;
    }

    // Numerical IK — reimplemented using class Jacob/PoE
    bool IKNum(const PosQuat& target,
               VectorXd& theta,
               int& cur_iter,
               double eomg = 1e-7,
               double ev   = 1e-7,
               int    max_iter = 200,
               double lambda   = 0.0,
               double step_clip = 0.0,
               bool   clamp_angles_pi = true) const
    {
        const int n = static_cast<int>(screws_.screw_list.cols());
        if (theta.size() != n) {
            throw std::invalid_argument("[RobotKinematics::IKNum] theta size mismatch");
        }
        auto compute_error = [&](const PosQuat& pos_quat_cur)->Vector6d {
            PosQuat pos_quat_cur_d = RM::PosQuats2RelativePosQuat(pos_quat_cur, target);
            return RM::PosQuat2Posso3(pos_quat_cur_d); // [v; w]
        };

        PosQuat pos_quat_fk = FKPoE(theta);
        Vector6d V_e = compute_error(pos_quat_fk);
        Vector3d v_lin = V_e.head<3>();
        Vector3d w_ang = V_e.tail<3>();
        bool err = (w_ang.norm() > eomg || v_lin.norm() > ev);

        int iter = 0;
        while (err && iter < max_iter) {
            MatrixXd Je = Jacob(theta); // 6 x n
            VectorXd dtheta(n);
            if (lambda > 0.0) {
                MatrixXd A = Je * Je.transpose() + (lambda * lambda) * MatrixXd::Identity(6,6);
                dtheta = Je.transpose() * A.inverse() * V_e;
            } else {
                dtheta = Je.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(V_e);
            }
            if (step_clip > 0.0) {
                double max_step = dtheta.cwiseAbs().maxCoeff();
                if (max_step > step_clip) dtheta *= (step_clip / max_step);
            }
            theta += dtheta;
            pos_quat_fk = FKPoE(theta);
            V_e = compute_error(pos_quat_fk);
            v_lin = V_e.head<3>();
            w_ang = V_e.tail<3>();
            err = (w_ang.norm() > eomg || v_lin.norm() > ev);
            if (clamp_angles_pi) {
                for (int i = 0; i < n; ++i) theta(i) = RM::ConstrainedAngle(theta(i), true);
            }
            ++iter;
        }
        cur_iter = iter;
        return !err;
    }

    // Manipulability (Yoshikawa) — implement here (no overloads)
    static double ManipulabilityIndex(const MatrixXd& J) {
        if (J.size() == 0) return 0.0;
        Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const VectorXd& s = svd.singularValues();
        double w = 1.0;
        for (int i = 0; i < s.size(); ++i) { w *= s[i]; if (w == 0.0) break; }
        return w;
    }
    static double ManipulabilityIndexLog10(const MatrixXd& J) {
        if (J.size() == 0) return -std::numeric_limits<double>::infinity();
        Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const VectorXd& s = svd.singularValues();
        double sum_log10 = 0.0;
        for (int i = 0; i < s.size(); ++i) {
            if (s[i] <= 0.0) return -std::numeric_limits<double>::infinity();
            sum_log10 += std::log10(s[i]);
        }
        return sum_log10;
    }
    static double MinSingularValue(const MatrixXd& J) {
        if (J.size() == 0) return 0.0;
        Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const VectorXd& s = svd.singularValues();
        return (s.size() ? s.minCoeff() : 0.0);
    }
    static bool NearSingular(const MatrixXd& J, double sigma_min_threshold = std::pow(10.0, -5.0)) {
        return RM::NearZero(ManipulabilityIndex(J), sigma_min_threshold);
    }

    double ManipulabilityIndex(const VectorXd& theta) const {
        return ManipulabilityIndex(Jacob(theta));
    }
    double ManipulabilityIndexLog10(const VectorXd& theta) const {
        return ManipulabilityIndexLog10(Jacob(theta));
    }
    double MinSingularValue(const VectorXd& theta) const {
        return MinSingularValue(Jacob(theta));
    }
    bool NearSingular(const VectorXd& theta, double sigma_min_threshold = std::pow(10.0, -5.0)) const {
        return NearSingular(Jacob(theta), sigma_min_threshold);
    }

    // Manipulability gradient (analytical PoE body Jacobian derivative)
    static VectorXd ManipulabilityGradient(const MatrixXd& J, double lambda = 0.0) {
        const int m = static_cast<int>(J.rows());
        const int n = static_cast<int>(J.cols());
        if (m == 0 || n == 0) return VectorXd();
        if (m != 6) {
            throw std::invalid_argument("[RobotKinematics::ComputeManipulabilityGradientFromJ] J must be 6×n");
        }
        const double w = ManipulabilityIndex(J);
        VectorXd grad = VectorXd::Zero(n);
        if (w == 0.0) return grad;
        MatrixXd Jdagger;
        if (lambda > 0.0) {
            MatrixXd A = J * J.transpose() + (lambda * lambda) * MatrixXd::Identity(m, m);
            Jdagger = J.transpose() * A.inverse();
        } else {
            Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
            const MatrixXd& U = svd.matrixU();
            const MatrixXd& V = svd.matrixV();
            const VectorXd& S = svd.singularValues();
            const int r = static_cast<int>(S.size());
            MatrixXd SigmaPlus = MatrixXd::Zero(r, r);
            const double eps = std::numeric_limits<double>::epsilon();
            const double maxS = (r > 0 ? S(0) : 0.0);
            const double tol = std::max(m, n) * eps * maxS;
            for (int i = 0; i < r; ++i) if (S(i) > tol) SigmaPlus(i, i) = 1.0 / S(i);
            Jdagger = V * SigmaPlus * U.transpose();
        }
        auto ad_matrix = [&](const Vector6d& Svec) -> Matrix6d {
            const Vector3d v = Svec.head<3>();
            const Vector3d wv = Svec.tail<3>();
            Matrix6d ad; ad.setZero();
            ad.topLeftCorner<3,3>()  = RM::R3Vec2so3Mat(wv);
            ad.topRightCorner<3,3>() = RM::R3Vec2so3Mat(v);
            ad.bottomRightCorner<3,3>() = RM::R3Vec2so3Mat(wv);
            return ad;
        };
        for (int j = 0; j < n; ++j) {
            if (j == 0) { grad(j) = 0.0; continue; }
            const Matrix6d ad_Jj = ad_matrix(J.col(j));
            double tr_sum = 0.0;
            for (int i = 0; i < j; ++i) {
                const Vector6d hij = -(ad_Jj * J.col(i));
                tr_sum += Jdagger.row(i).dot(hij);
            }
            grad(j) = w * tr_sum;
        }
        return grad;
    }
    VectorXd ManipulabilityGradient(const VectorXd& theta, double lambda = 0.0) const {
        MatrixXd Jb = Jacob(theta);
        return ManipulabilityGradient(Jb, lambda);
    }

private:
    int n_ {0};
    DHTable   dh_table_;   // Optional: valid if constructed from DH
    ScrewList screws_;
};

#endif // ROBOT_KINEMATICS_UTILS_HPP
