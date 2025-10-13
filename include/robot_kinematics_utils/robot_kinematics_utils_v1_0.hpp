#ifndef ROBOT_KINEMATICS_UTILS_HPP
#define ROBOT_KINEMATICS_UTILS_HPP

// Author: Wei-Hsuan Cheng, johnathancheng0125@gmail.com, https://github.com/wei-hsuan-cheng
// GitHub repo: https://github.com/wei-hsuan-cheng/robot_math_utils
// v1_0, last edit: 251010
//
// Version history:
//  - Migrate the kinematics-related functions from robot_math_utils_v1_16.hpp to this new file.

#include "robot_math_utils/robot_math_utils_v1_17.hpp"
#include <limits>

// Short alias for the math/PoE utility
using RM = RMUtils;

// RKUtils: robot kinematics object-oriented wrapper around RMUtils kinematics.
// Holds a robot model (DH table and/or PoE screw list) and exposes
// FK / Jacobian / IK and manipulability utilities as methods.

struct DHParams {
    // Modified Denavit-Hartenberg (D-H) parameters for each joint
    // alpha{i-1}, a{i-1}, d{i}, theta{i}
    double alpha;
    double a;
    double d;
    double theta;
    // Constructor from individual parameters
    DHParams(double alpha, double a, double d, double theta)
        : alpha(alpha), a(a), d(d), theta(theta) {}
    // Constructor from Vector4d
    DHParams(const Eigen::Vector4d& dh_params)
        : alpha(dh_params(0)), a(dh_params(1)), d(dh_params(2)), theta(dh_params(3)) {}
};

struct DHTable {
    std::vector<DHParams> joints;  // Store D-H parameters for each joint
    Eigen::MatrixXd dh_table;      // Store D-H table as a matrix

    std::string robot_name = "arm";              // default: "arm"
    std::string base_frame = "base";
    std::string ee_frame   = "ee";
    std::vector<std::string> joint_names;                 // default empty
    MatrixXd joint_limits = MatrixXd::Zero(0,4); // n×4: [ll, ul, vel, eff]

    // Default constructor: initialize with an empty vector and an empty matrix.
    DHTable()
      : joints(), dh_table(MatrixXd::Zero(0, 4)) {}

    // Constructor to initialise DHTable from a vector of DHParams (joints)
    DHTable(const std::vector<DHParams>& joints)
        : joints(joints), dh_table(joints.size(), 4) {
        // Fill the table with the D-H parameters from the joints
        for (size_t i = 0; i < joints.size(); ++i) {
            dh_table(i, 0) = joints[i].alpha;
            dh_table(i, 1) = joints[i].a;
            dh_table(i, 2) = joints[i].d;
            dh_table(i, 3) = joints[i].theta;
        }
    }

    void setMeta(const std::string& robot_name_,
                 const std::string& base_frame_,
                 const std::string& ee_frame_,
                 const std::vector<std::string>& joint_names_,
                 const MatrixXd& joint_limits_) {
        robot_name = robot_name_;
        base_frame = base_frame_.empty() ? "base" : base_frame_;
        ee_frame   = ee_frame_.empty()   ? "ee"   : ee_frame_;
        // joint names: keep if same size, else generate defaults j1..jn
        const int n = static_cast<int>(dh_table.rows());
        if ((int)joint_names_.size() == n) {
            joint_names = joint_names_;
        } else {
            joint_names.resize(n);
            for (int i = 0; i < n; ++i) joint_names[i] = "j" + std::to_string(i+1);
        }
        // joint limits: accept n×4, else fallback to zeros
        if (joint_limits_.rows() == n && joint_limits_.cols() == 4) {
            joint_limits = joint_limits_;
        } else {
            joint_limits = MatrixXd::Zero(n, 4);
        }
    }

    /**
     * @brief Print the D-H table in formatted columns with dividers
     */
    void PrintTable() const {

        // Add joint index (the i'th joint) to the D-H table
        int n = dh_table.rows(); // number of joints
        Eigen::MatrixXd dh_table_with_joint_index(n, 5);
        // first column = 0,1,...,n-1
        for(int i = 0; i < n; ++i){
            dh_table_with_joint_index(i, 0) = i+1; // 1-based index for each joint
            dh_table_with_joint_index.row(i).segment<4>(1) = dh_table.row(i);
        } // now dh_table_with_joint_index has columns [i | alpha a d theta]

        int rows = dh_table_with_joint_index.rows();
        int cols = dh_table_with_joint_index.cols();  // should be 5: i, alpha_{i-1}, a_{i-1}, d_{i}, theta_{i}

        // Print title box
        const int width = 12;
        const std::string& title = "D-H table (modified D-H parameters)";
        const std::string& subtitle = "D-H transform: T_{i-1}_{i} = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Trans_z(d_{i}) * Rot_z(theta_{i})";
        int inner = std::max(title.size(), subtitle.size());
        int boxW = inner + 12;

        std::cout << "\n";
        std::cout << std::string(boxW, '*') << "\n";
        std::cout << "***** " << title << std::string(inner - title.size(), ' ') << " *****\n";
        std::cout << "***** " << subtitle << std::string(inner - subtitle.size(), ' ') << " *****\n";
        std::cout << std::string(boxW, '*') << "\n";

        // Metadata
        if (!robot_name.empty()) {
            std::cout << "robot_name: " << robot_name << "\n";
        }
        if (!base_frame.empty() || !ee_frame.empty()) {
            std::cout << "base_frame: " << base_frame << "\nee_frame: " << ee_frame << "\n";
        }
        if (!joint_names.empty()) {
            std::cout << "joints(n=" << joint_names.size() << "): ";
            for (size_t i = 0; i < joint_names.size(); ++i) {
                std::cout << (i ? ", " : "") << joint_names[i];
            }
            std::cout << "\n";
        }

        // Print joint limits
        std::cout << "\n-- Joint limits [ll, ul, vel, eff] [rad, rad, rad/s, Nm] -->\n";
        for (int i = 0; i < joint_names.size(); ++i) {
        std::cout << "  " << joint_names[i] << ": ["
                    << joint_limits(i,0) << ", "
                    << joint_limits(i,1) << ", "
                    << joint_limits(i,2) << ", "
                    << joint_limits(i,3) << "]\n";
        }
        std::cout << std::endl;

        // Header separator
        std::cout << "|";
        for (int c = 0; c < cols; ++c) std::cout << std::string(width, '-') << "|";
        std::cout << "\n";

        // Header
        std::vector<std::string> hdr = {"i", "alpha_{i-1}", "a_{i-1}", "d_{i}", "theta_{i}"};
        std::cout << "|";
        for (const auto& h : hdr) {
            std::cout << std::setw(width) << h << "|";
        }
        std::cout << "\n";

        // Separator
        std::cout << "|";
        for (int c = 0; c < cols; ++c) std::cout << std::string(width, '-') << "|";
        std::cout << "\n";

        // Data rows
        for (int r = 0; r < rows; ++r) {
            std::cout << "|";
            for (int c = 0; c < cols; ++c) {
                int digits = (c == 0) ? 0 : 4; // first column (joint index) has no decimal places
                std::cout << std::setw(width) << std::fixed << std::setprecision(digits)
                          << dh_table_with_joint_index(r, c) << "|";
            }
            std::cout << "\n";
        }

        // Bottom separator
        std::cout << "|";
        for (int c = 0; c < cols; ++c) std::cout << std::string(width, '-') << "|";
        std::cout << "\n";

        std::cout << std::fixed; // reset format
    }

    // Function to print a specific joint's D-H parameters
    void PrintJoint(int joint_index) const {
        if (joint_index < 1 || joint_index > joints.size()) {
            throw std::out_of_range("Invalid joint number.");
        }
        const DHParams& joint = joints[joint_index - 1]; // 1-based index
        std::cout << "j" << joint_index << ": alpha = " << joint.alpha
                  << ", a = " << joint.a
                  << ", d = " << joint.d
                  << ", theta = " << joint.theta << std::endl;
    }

    // Function to access a specific joint's D-H parameters
    const DHParams& GetJoint(int joint_index) const {
        if (joint_index < 1 || joint_index > joints.size()) {
            throw std::out_of_range("Invalid joint number.");
        }
        return joints[joint_index - 1]; // 1-based index
    }
};

// Overload operator<< to print the D-H table
std::ostream& operator<<(std::ostream& os, const DHTable& table) {
    os << table.dh_table;
    return os;
}

/**
 * @brief List of screw axes and home configuration for FK/IK using PoE
 */
struct ScrewList {
    // ==== Existing fields (unchanged) ====
    Eigen::MatrixXd screw_list;  // 6×n, each column [v; w]
    PosQuat M;                   // home pose

    // ==== New metadata attributes (safe defaults) ====
    std::string robot_name = "arm";              // default: "arm"
    enum class Rep { Space, Body };
    Rep screw_representation = Rep::Body;        // default: body representation
    std::vector<std::string> joint_names;        // default: empty
    std::string base_frame = "base";        // default
    std::string ee_frame   = "ee";          // default
    Eigen::MatrixXd joint_limits = Eigen::MatrixXd::Zero(0,4); // n×4: [lower, upper, vel, effort]

    // ==== Old constructors (kept, for backward compatibility) ====
    ScrewList()
      : screw_list(Eigen::MatrixXd::Zero(6, 0)),
        M(PosQuat()) {}

    ScrewList(const Eigen::MatrixXd& screw_list_, const PosQuat& M_)
      : screw_list(screw_list_), M(M_) {}

    // ==== New constructor with all metadata ====
    ScrewList(const Eigen::MatrixXd& screw_list_, const PosQuat& M_,
              const std::string& robot_name_,
              Rep rep_,
              const std::vector<std::string>& joint_names_,
              const std::string& base_frame_,
              const std::string& ee_frame_,
              const Eigen::MatrixXd& joint_limits_)
      : screw_list(screw_list_), M(M_),
        robot_name(robot_name_), screw_representation(rep_),
        joint_names(joint_names_), base_frame(base_frame_), ee_frame(ee_frame_) {
        setJointLimits(joint_limits_);
    }

    // Parse "space"/"body" string to enum
    static Rep ParseRep(const std::string& rep_str) {
        std::string s = rep_str;
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        if (s == "space") return Rep::Space;
        return Rep::Body;
    }

    // Safely set joint limits (resize fallback to zero matrix if mismatch)
    void setJointLimits(const MatrixXd& jl) {
        const int n = static_cast<int>(screw_list.cols());
        if (jl.rows() == n && jl.cols() == 4) {
            joint_limits = jl;
        } else {
            joint_limits = MatrixXd::Zero(n, 4); // fallback
        }
    }

    // Fill metadata in one call (convenience for ROS2 node setup)
    void setMeta(const std::string& robot_name_,
                 Rep rep_,
                 const std::vector<std::string>& joint_names_,
                 const std::string& base_frame_,
                 const std::string& ee_frame_,
                 const MatrixXd& joint_limits_) {
        robot_name = robot_name_;
        screw_representation = rep_;
        joint_names = joint_names_;
        base_frame = base_frame_;
        ee_frame   = ee_frame_;
        setJointLimits(joint_limits_);
    }

    /**
     * @brief Print screw list with metadata and home pose
     */
    void PrintList() const {
        const int cols = screw_list.cols();

        // Title box
        const int width = 12;
        const std::string title    = "Screw list (end-effector frame screw axes)";
        const std::string subtitle = "S_e,i = [v, w]^T, i = 1...n";
        const int inner = static_cast<int>(std::max(title.size(), subtitle.size()));
        const int boxW  = inner + 12;

        std::cout << "\n" << std::string(boxW, '*') << "\n";
        std::cout << "***** " << title    << std::string(inner - title.size(),    ' ') << " *****\n";
        std::cout << "***** " << subtitle << std::string(inner - subtitle.size(), ' ') << " *****\n";
        std::cout << std::string(boxW, '*') << "\n";

        // Metadata
        if (!robot_name.empty()) {
            std::cout << "robot_name: " << robot_name << "\n";
        }
        std::cout << "representation: " << (screw_representation == Rep::Body ? "body" : "space") << "\n";
        if (!base_frame.empty() || !ee_frame.empty()) {
            std::cout << "base_frame: " << base_frame << "\nee_frame: " << ee_frame << "\n";
        }
        if (!joint_names.empty()) {
            std::cout << "joints(n=" << joint_names.size() << "): ";
            for (size_t i = 0; i < joint_names.size(); ++i) {
                std::cout << (i ? ", " : "") << joint_names[i];
            }
            std::cout << "\n";
        }

        // Print joint limits
        std::cout << "\n-- Joint limits [ll, ul, vel, eff] [rad, rad, rad/s, Nm] -->\n";
        for (int i = 0; i < joint_names.size(); ++i) {
        std::cout << "  " << joint_names[i] << ": ["
                    << joint_limits(i,0) << ", "
                    << joint_limits(i,1) << ", "
                    << joint_limits(i,2) << ", "
                    << joint_limits(i,3) << "]\n";
        }
        std::cout << std::endl;

        // Table header separator
        std::cout << "|";
        for (int i = 0; i < cols; ++i) std::cout << std::string(width, '-') << "|";
        std::cout << "\n";

        // Header row (S_e,i)
        std::cout << "|";
        for (int i = 0; i < cols; ++i) {
            std::ostringstream oss;
            oss << std::setw(width) << ("S_e," + std::to_string(i + 1));
            std::cout << oss.str() << "|";
        }
        std::cout << "\n";

        // Separator
        std::cout << "|";
        for (int i = 0; i < cols; ++i) std::cout << std::string(width, '-') << "|";
        std::cout << "\n";

        // Data rows
        for (int r = 0; r < 6; ++r) {
            std::cout << "|";
            for (int c = 0; c < cols; ++c) {
                std::ostringstream oss;
                oss << std::setw(width) << std::fixed << std::setprecision(4) << screw_list(r, c);
                std::cout << oss.str() << "|";
            }
            std::cout << "\n";
        }

        // Bottom separator
        std::cout << "|";
        for (int i = 0; i < cols; ++i) std::cout << std::string(width, '-') << "|";
        std::cout << "\n";

        // Print home pose
        std::cout << "\n-- Home pose M [m, quat_wxyz] -->\n"
                  << M.pos.x()   << ", "
                  << M.pos.y()   << ", "
                  << M.pos.z()   << ", "
                  << M.quat.w()  << ", "
                  << M.quat.x()  << ", "
                  << M.quat.y()  << ", "
                  << M.quat.z()  << "\n";
    }
};


class RKUtils {
public:
    // Constructors
    explicit RKUtils(const DHTable& dh)
    : dh_table_(dh), screws_(ScrewListFromDH(dh))
    {
        n_ = static_cast<int>(screws_.screw_list.cols());
        q_.setZero(n_);
        qd_.setZero(n_);
        UpdateRobotState();
    }

    explicit RKUtils(const ScrewList& screws)
    : screws_(screws)
    {
        n_ = static_cast<int>(screws_.screw_list.cols());
        q_.setZero(n_);
        qd_.setZero(n_);
        UpdateRobotState();
    }

    // Accessors
    int dof() const { return n_; }
    const DHTable&    dh()     const { return dh_table_; }
    const ScrewList&  screws() const { return screws_; }

    // State accessors
    const VectorXd& q()   const { return q_; }
    const VectorXd& qd()  const { return qd_; }
    const PosQuat&  pose() const { return pos_quat_b_e_; }
    const Vector6d& twist() const { return twist_e_; }
    const MatrixXd& jacob() const { return jacob_; }
    double manipulability() const { return w_; }
    const VectorXd& manipulabilityGradient() const { return dw_dq_; }
    double manipulabilityVelocity() const { return wd_; }

    // Update robot state: uses current q_/qd_
    void UpdateRobotState() {
        if (q_.size() != n_) q_.setZero(n_);
        if (qd_.size() != n_) qd_.setZero(n_);
        pos_quat_b_e_ = FKPoE(q_);
        jacob_ = Jacob(q_);
        twist_e_ = jacob_ * qd_;
        w_ = ManipulabilityIndex(jacob_);
        dw_dq_ = ManipulabilityGradient(jacob_);
        wd_ = dw_dq_.transpose() * qd_;
    }

    // Update robot state with provided q and qd
    void UpdateRobotState(const VectorXd& q_in, const VectorXd& qd_in) {
        if (q_in.size() != n_) {
            throw std::invalid_argument("[RKUtils::UpdateRobotState] q size mismatch");
        }
        if (qd_in.size() != n_) {
            throw std::invalid_argument("[RKUtils::UpdateRobotState] qd size mismatch");
        }
        q_ = q_in;
        qd_ = qd_in;
        UpdateRobotState();
    }

    // Forward kinematics (FK)
    /**
     * @brief Compute the homogeneous D-H transformation for a single joint.
     * @param alpha rotation about x-axis (rad)
     * @param a     translation along x-axis (m)
     * @param d     translation along z-axis (m)
     * @param theta rotation about z-axis (rad)
     * @return      Pose as PosQuat
     */
    static PosQuat DHTransform(double alpha, double a, double d, double theta) {
        // Rotation about x-axis by alpha
        PosQuat t1;
        t1.pos  = Vector3d::Zero();
        t1.quat = RM::Quatx(alpha);

        // Translation along x-axis by a
        PosQuat t2;
        t2.pos  = Vector3d(a, 0.0, 0.0);
        t2.quat = Quaterniond::Identity();

        // Translation along z-axis by d
        PosQuat t3;
        t3.pos  = Vector3d(0.0, 0.0, d);
        t3.quat = Quaterniond::Identity();

        // Rotation about z-axis by theta
        PosQuat t4;
        t4.pos  = Vector3d::Zero();
        t4.quat = RM::Quatz(theta);

        // Combine: T = R_x(alpha) * Trans_x(a) * Trans_z(d) * R_z(theta)
        return RM::TransformPosQuats({t1, t2, t3, t4});
    }

    /**
     * @brief Overload: compute D-H transform from a DHParams struct
     * @param dh    D-H parameters (alpha, a, d, theta)
     * @return      Pose as PosQuat
     */
    static PosQuat DHTransform(const DHParams& dh) {
        return DHTransform(dh.alpha, dh.a, dh.d, dh.theta);
    }

    /**
     * @brief Compute the forward kinematics (FK) for a manipulator using D-H parameters,
     *        adding joint offsets to the default theta values from the table.
     * @param theta_list   Joint offsets [rad] to add to the default theta values; size must match number of joints
     * @return             Combined end-effector pose as PosQuat
     */
    PosQuat FKDH(const VectorXd& theta_list) const {
        // Number of joints
        size_t n = dh_table_.joints.size();
        if (static_cast<size_t>(theta_list.size()) != n) {
            throw std::invalid_argument("[RKUtils::FKDH] theta_list size mismatch");
        }
        // Compose transforms for each joint using D-H parameters
        std::vector<PosQuat> transforms;
        transforms.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const auto& dh = dh_table_.joints[i];
            double theta = dh.theta + theta_list(static_cast<int>(i));
            transforms.push_back(DHTransform(dh.alpha, dh.a, dh.d, theta));
        }
        return RM::TransformPosQuats(transforms);
    }

    /**
     * @brief Compute forward kinematics by D-H at zero configuration and generate ScrewList
     * @return      ScrewList with screw axes in end-effector frame and home pose
     */
    static ScrewList ScrewListFromDH(const DHTable& table) {
        // number of joints
        size_t n = table.joints.size();
        // zero theta offsets (implicit by using default DH thetas)
        // compute zero configuration pose by chaining D-H transforms at default thetas
        std::vector<PosQuat> transforms_zero;
        transforms_zero.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const auto& dh = table.joints[i];
            transforms_zero.push_back(DHTransform(dh));
        }
        PosQuat zero_config_pose = RM::TransformPosQuats(transforms_zero);
        Matrix4d T_b_e_zero = RM::PosQuat2TMat(zero_config_pose);

        // compute pose of joint's local frame w.r.t. base
        std::vector<Eigen::Matrix4d> T_b_ji_list;
        T_b_ji_list.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            std::vector<PosQuat> partial;
            partial.reserve(i + 1);
            for (size_t j = 0; j <= i; ++j) {
                const auto& dh = table.joints[j];
                partial.push_back(DHTransform(dh.alpha, dh.a, dh.d, dh.theta));
            }
            PosQuat pq_i = RM::TransformPosQuats(partial);
            T_b_ji_list.push_back(RM::PosQuat2TMat(pq_i));
        }

        // build screw_list in end-effector frame
        Eigen::MatrixXd screw_list(6, n);
        for (size_t i = 0; i < n; ++i) {
            // pose of joint frame w.r.t. end-effector
            Eigen::Matrix4d T_e_ji = T_b_e_zero.inverse() * T_b_ji_list[i];
            Eigen::Vector3d u_hat = RM::TMat2PosRot(T_e_ji).rot.col(2); // axis of rotation: z-axis of joint frame w.r.t. ee
            Eigen::Vector3d r = -RM::TMat2PosRot(T_e_ji).pos; // rotation radius vector: (negative) position of joint frame w.r.t. ee
            Eigen::Vector3d v = u_hat.cross(r); // linear velocity: v = u_hat x r

            Vector6d S;
            S.head<3>() = v;
            S.tail<3>() = u_hat;
            screw_list.col(i) = S;
        }

        // ---- construct ScrewList (keeps old constructor) ----
        ScrewList out(screw_list, zero_config_pose);

        // ---- inherit metadata from DHTable (safe defaults) ----
        // joint names: use table’s if size matches; otherwise j1..jn
        std::vector<std::string> jnames;
        if (table.joint_names.size() == n) {
            jnames = table.joint_names;
        } else {
            jnames.resize(n);
            for (size_t i = 0; i < n; ++i) jnames[i] = "j" + std::to_string(i + 1);
        }

        // joint limits: use n×4 from table; otherwise zeros
        MatrixXd jl;
        if (table.joint_limits.rows() == static_cast<int>(n) &&
            table.joint_limits.cols() == 4) {
            jl = table.joint_limits;
        } else {
            jl = MatrixXd::Zero(static_cast<int>(n), 4);
        }

        // representation hint: keep body as default (no behavioral change)
        const auto rep = ScrewList::Rep::Body;

        // apply metadata to ScrewList (non-throwing)
        out.setMeta(
            table.robot_name,      // robot_name ("" OK)
            rep,                   // representation (Body default)
            jnames,                // joint_names (size-safe)
            table.base_frame,      // base frame ("base" default from DHTable)
            table.ee_frame,        // ee frame ("ee" default from DHTable)
            jl                     // joint limits (n×4 or zeros)
        );

        return out;
    }

    /**
     * @brief PoE forward kinematics: T = M * exp([S_e,1]θ1) * ... * exp([S_e,n]θn),
     *        where S_e,i is the i'th screw axis in end-effector frame.
     * @param theta_list Joint angle offsets [rad], size n
     * @return           End-effector pose as PosQuat
     */
    PosQuat FKPoE(const VectorXd& theta_list) const {
        const int n = static_cast<int>(theta_list.size());
        if (screws_.screw_list.cols() != n) {
            throw std::invalid_argument("[RKUtils::FKPoE] theta_list size mismatch");
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

    // Differential kinematics (DK)
    // Jacobian matrix J_e(θ) in end-effector frame for PoE with end-effector screws S_e,i
    // screws.screw_list: 6 x n matrix (each column is a end-effector frame screw axis S_e,i)
    // theta_list: n x 1 joint vector
    // Returns: 6 x n J_e
    MatrixXd Jacob(const VectorXd& theta_list) const {
        const int n = static_cast<int>(screws_.screw_list.cols());
        if (theta_list.size() != n) {
            throw std::invalid_argument("[RKUtils::Jacob] theta_list size mismatch");
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

    // Numerical inverse kinematics (IK)
    // Solve for theta_list s.t. FKPoE(screws, theta_list) ≈ target.
    // - Calculated by Jacobian in end-effector/body frame via PoE
    // - Error twist V_e = se3Vec( log( T_fk^{-1} * T_target ) )  (linear first, then angular)
    // - Solver: least-squares via SVD; optional DLS (lambda>0), step clipping, angle wrapping
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
            throw std::invalid_argument("[RKUtils::IKNum] theta size mismatch");
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

    // Collision/singularity check
    // Yoshikawa manipulability index = product of singular values of J. 
    // Paper: T. Yoshikawa, “Manipulability of robotic mechanisms,” The International Journal of Robotics Research, vol. 4, pp. 3–9, 1985.
    // If any of the singular values is 0, return 0 (singular).
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

    // Manipulability gradient (Yoshikawa) for a 6×n body/end-effector Jacobian J_b.
    // Returns ∂w/∂θ ∈ R^n using: (∂w/∂θ_j) = w · Tr( H^{(j)} · J_b^{†} ),
    // where H^{(j)} = ∂J_b/∂θ_j has columns: for i < j, H^{(j)}[:, i] = − ad_{J_j} J_i; else 0.
    // Notes
    //  - This uses the PoE body-Jacobian derivative identities and requires only J_b itself.
    //  - J_b^{†} is computed via SVD by default; set lambda>0 for damped least-squares J^{†} = Jᵀ (J Jᵀ + λ² I)^{-1}.
    //  - If J is empty or w == 0, returns zero vector of length n.
    static VectorXd ManipulabilityGradient(const MatrixXd& J, double lambda = 0.0) {
        const int m = static_cast<int>(J.rows());
        const int n = static_cast<int>(J.cols());
        if (m == 0 || n == 0) return VectorXd();
        if (m != 6) {
            throw std::invalid_argument("[RKUtils::ComputeManipulabilityGradientFromJ] J must be 6×n");
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

        for (int j = 0; j < n; ++j) {
            if (j == 0) { grad(j) = 0.0; continue; }
            const Matrix6d ad_Jj = RM::adjoint(J.col(j));
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

    // Runtime state
    VectorXd q_;
    VectorXd qd_;
    PosQuat  pos_quat_b_e_;
    Vector6d twist_e_ {Vector6d::Zero()};
    MatrixXd jacob_;
    double   w_ {0.0};
    VectorXd dw_dq_;
    double wd_{0.0};
};

#endif // ROBOT_KINEMATICS_UTILS_HPP
