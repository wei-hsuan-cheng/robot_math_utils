#ifndef ROBOT_MATH_UTILS_HPP
#define ROBOT_MATH_UTILS_HPP

// Author: Wei-Hsuan Cheng, johnathancheng0125@gmail.com, https://github.com/wei-hsuan-cheng
// GitHub repo: https://github.com/wei-hsuan-cheng/robot_math_utils
// ver_1.12, last edit: 250729
//
// Version history:
//  2025-07-20 v1.12
//    - Added ScrewList, ScrewListFromDH, FKDH, FKPoE for forward kinematics (FK) 
//      using D-H parameters and product of exponentials (PoE) method.


#include <Eigen/Dense>
#include <Eigen/Geometry> // For Quaternion
#include <deque>
#include <utility>
#include <fstream>
#include <iomanip>  // For setting date format
#include <chrono>   // For timestamp
#include <ctime>    // For time conversion
#include <cmath>
#include <random> // For random number generation
#include <unsupported/Eigen/MatrixFunctions>  // For matrix logarithm and exponential
#include <iostream>

using Eigen::Quaterniond;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Eigen::VectorXd;


struct PosQuat {
    Vector3d pos;
    Quaterniond quat;
    PosQuat() : pos(Vector3d::Zero()), quat(Quaterniond::Identity()) {}
    PosQuat(const Vector3d& pos, const Quaterniond& quat)
        : pos(pos), quat(quat) {}
};

struct PosRot {
    Vector3d pos;
    Matrix3d rot;
    PosRot() : pos(Vector3d::Zero()), rot(Matrix3d::Identity()) {}
    PosRot(const Vector3d& pos, const Matrix3d& rot)
        : pos(pos), rot(rot) {}
};

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

    // Default constructor: initialize with an empty vector and an empty matrix.
    DHTable()
      : joints(), dh_table(Eigen::MatrixXd::Zero(0, 4)) {}

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
 * @brief List of screw axes and home configuration for forward kinematics (FK) based on product of exponentials (PoE) method
 */
struct ScrewList {
    /// 6×n matrix, each column is an end-effector frame screw axis [v; w]
    Eigen::MatrixXd screw_list;
    /// Home pose of the end-effector
    PosQuat M;

    /// Default constructor
    ScrewList()
      : screw_list(Eigen::MatrixXd::Zero(6, 0)),
        M(PosQuat()) {}

    /**
     * @brief Construct with screw axis matrix and home pose
     * @param screw_list_ 6×n screw axis matrix
     * @param M_          Home pose as PosQuat
     */
    ScrewList(const Eigen::MatrixXd& screw_list_, const PosQuat& M_)
      : screw_list(screw_list_), M(M_) {}

    /**
     * @brief Print the screw axes with formatted columns and dividers
     */
    void PrintList() const {
        const int cols = screw_list.cols();

        // Print title box
        const int width = 12;
        const std::string& title = "Screw list (end-effector frame screw axes)";
        const std::string& subtitle = "S_e,i = [v, w]^T (se(3) or R^6 vector), i = 1, ..., n";
        int inner = std::max(title.size(), subtitle.size());
        int boxW = inner + 12;

        std::cout << "\n";
        std::cout << std::string(boxW, '*') << "\n";
        std::cout << "***** " << title << std::string(inner - title.size(), ' ') << " *****\n";
        std::cout << "***** " << subtitle << std::string(inner - subtitle.size(), ' ') << " *****\n";
        std::cout << std::string(boxW, '*') << "\n";

        // Header separator
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

        std::cout << std::fixed; // reset format
    }
};



class RMUtils {
public:
    /* Unit conversion */
    static constexpr double r2d = 180.0 / M_PI;
    static constexpr double d2r = M_PI / 180.0;

    /* Data logging */
    template<typename VectorType>
    static void PrintVec(const VectorType& vec, const std::string& vec_name) {
        std::cout << vec_name << " = ";
        for (int i = 0; i < vec.size(); ++i) {
            std::cout << vec[i];
            if (i < vec.size() - 1) {
                std::cout << ", ";  // Add comma between elements
            }
        }
        std::cout << std::endl;
    }

    static void InitDatalog(std::ofstream& csv_writer_, const std::string& datalog_filename, const std::vector<std::string>& header_row) {
        csv_writer_.open(datalog_filename);
        if (csv_writer_.is_open()) {
            // Write the header row from the provided vector of strings
            for (size_t i = 0; i < header_row.size(); ++i) {
                csv_writer_ << header_row[i];
                if (i < header_row.size() - 1) {
                    csv_writer_ << ",";  // Add a comma separator between header entries
                }
            }
            csv_writer_ << "\n";  // End the header row
        }
    }

    template<typename VectorType>
    static void Datalog(std::ofstream& csv_writer_, std::_Put_time<char> datetime, int k, double Ts, double time_elapsed, const std::vector<VectorType>& data_vectors) {
        if (!csv_writer_.is_open()) return;

        // Start writing the timestamp and elapsed time
        csv_writer_ << datetime  << "," << k << "," << k * Ts << "," << time_elapsed; // datetime, k [idx], kTs [s], t [s] 
        for (const auto& vec : data_vectors) {
            for (int i = 0; i < vec.size(); ++i) {
                csv_writer_ << "," << vec[i];  // Add each element separated by a comma
            }
        }
        csv_writer_ << "\n"; // End of each row
    }

    template<typename MatrixType>
    static void SaveMat(const MatrixType& mat, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            for (int i = 0; i < mat.rows(); ++i) {
                for (int j = 0; j < mat.cols(); ++j) {
                    file << mat(i, j);
                    if (j < mat.cols() - 1) {
                        file << ","; // Add a comma unless it's the last column
                    }
                }
                file << "\n"; // Newline after each row
            }
            file.close();
        } else {
            std::cerr << "Could not open file " << filename << " for writing." << std::endl;
        }
    }


    /* Numerical conditions */
    static bool NearZero(const double val, double thresh = pow(10, -7)) {
        return (std::abs(val) < thresh);
    }

    static double ConstrainedAngle(double angle, bool rad = true) {
        // Normalize angle to (-pi, pi] or (-180, 180]
        double full_circle = rad ? 2.0 * M_PI : 360.0;
        double half_circle = rad ? M_PI : 180.0;
        // Use modulo operation to wrap angle within (-full_circle, full_circle)
        angle = std::fmod(angle, full_circle);
        // Adjust angle to be within (-half_circle, half_circle]
        if (angle <= -half_circle) {
            angle += full_circle;
        } else if (angle > half_circle) {
            angle -= full_circle;
        }
        return angle;
    }

    /* Sliding window functions */
    template<typename VectorType>
    static VectorType MeanBuffer(const std::deque<VectorType>& buffer) {
        /* Mean in a sliding window */
        VectorType mean = VectorType::Zero(buffer.front().size());
        for (const auto& vec : buffer) {
            mean += vec;
        }
        mean /= static_cast<double>(buffer.size());
        return mean;
    }

    template<typename VectorType>
    static VectorType StdBuffer(const std::deque<VectorType>& buffer) {
        /* Standard deviation in a sliding window */
        VectorType mean = MeanBuffer(buffer);
        VectorType variance = VectorType::Zero(buffer.front().size());
        for (const auto& vec : buffer) {
            VectorType diff = vec - mean;
            variance += diff.array().square().matrix();
        }
        variance /= static_cast<double>(buffer.size());
        return variance.array().sqrt();
    }

    template<typename VectorType>
    static VectorType MAvg(const VectorType& vec, std::deque<VectorType>& buffer, std::size_t window_size) {
        /* Moving average */
        buffer.push_back(vec);
        if (buffer.size() > window_size) {
            buffer.pop_front();
        } // Ensure buffer size is within window_size
        // Return the average over the available samples
        return MeanBuffer(buffer);
    }

    /* Basic math functions */
    static inline double ErrorPercentage(double meas, double gt) {
        return (meas - gt) / gt;
    }

    static double Sinc(double x) {
        return (NearZero(x)) ? cos(x) : std::sin(x) / x;
        // return (NearZero(x)) ? 1.0 : std::sin(x) / x;
    }

    static double ArcCos(double cos_val, bool rad = true) {
        if (cos_val > 1.0 || cos_val < -1.0) {
            if (cos_val > 1.0) cos_val = 1.0;
            if (cos_val < -1.0) cos_val = -1.0;
        }
        double theta = rad ? std::acos(cos_val) : std::acos(cos_val) * r2d;
        return theta; // std::acos() guarantees theta lies in [0, pi] [rad] or [0, 180] [deg]
    }

    template<typename VectorType>
    static double Norm(const VectorType& v) {
        return v.norm();
    }

    template<typename MatrixType>
    static double MatNorm(const MatrixType& M) {
        return M.norm();
    }

    template<typename VectorType>
    static VectorType Normalized(const VectorType& v) {
        return v.normalized();
    }

    template<typename MatrixType>
    static MatrixType Transpose(const MatrixType& M) {
        return M.transpose();
    }

    template<typename MatrixType>
    static double Tr(const MatrixType& M) {
        return M.trace();
    }

    template<typename MatrixType>
    static double Det(const MatrixType& M) {
        return M.determinant();
    }

    template<typename MatrixType>
    static MatrixType Inv(const MatrixType& M) {
        return M.inverse();
    }

    template<typename MatrixType>
    static MatrixType LeftPInv(const MatrixType& M) {
        return Inv(Transpose(M) * M) * Transpose(M);
    }

    // Random variables
    static double RandNorDist(double mean = 0.0, double stddev = 1.0) {
        static std::mt19937 rng(std::random_device{}()); // Standard mersenne_twister_engine
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng);
    }

    template<typename VectorType, typename MatrixType>
    static VectorType RandNorDistVec(const VectorType& mean, const MatrixType& cov) {
        static std::mt19937 rng(std::random_device{}());
        std::normal_distribution<double> dist;
        if (mean.size() != cov.rows() || cov.rows() != cov.cols()) {
            throw std::invalid_argument("[RMUtils::RandNorDistVec() Error] Mean vector size and covariance matrix dimensions must match.");
        }
        // Perform Cholesky decomposition (LLT) of the covariance matrix
        Eigen::LLT<MatrixType> lltOfCov(cov);
        if (lltOfCov.info() == Eigen::NumericalIssue) {
            throw std::runtime_error("Covariance matrix is not positive definite.");
        }
        MatrixType L = lltOfCov.matrixL();
        // Generate a vector of standard normal random variables
        VectorType z = VectorType::NullaryExpr(mean.size(), [&]() { return dist(rng); });
        return mean + L * z;
    }


    /* Robot transformation functions */
    // Homogeneous coordinates
    static Vector4d R3Vec2Homo(const Vector3d& v, double s = 1.0) {
        Vector4d v_h;
        v_h.head<3>() = v * s;
        v_h(3) = s;
        return v_h;
    }


    static std::vector<Vector4d> R3Vecs2Homos(const std::vector<Vector3d>& r3_vecs, double s = 1.0) {
        if (r3_vecs.empty()) {
            throw std::invalid_argument("[RMUtils::R3Vecs2Homos() Error] The input list of vectors is empty.");
        }
        std::vector<Vector4d> r3_vecs_h;
        for (const auto& r3_vec : r3_vecs) {
            Vector4d r3_vec_h = R3Vec2Homo(r3_vec, s);
            r3_vecs_h.push_back(r3_vec_h);
        }
        return r3_vecs_h; // {v1_h, v2_h, v3_h, ...}
    }

    static Vector3d Homo2R3Vec(const Vector4d& v_h) {
        if (v_h(3) == 0) {
            throw std::invalid_argument("[RMUtils::Homo2R3Vec() Error] The homogeneous coordinate (last element) must not be zero.");
        }
        return v_h.head<3>() / v_h(3);
    }

    static std::vector<Vector3d> Homos2R3Vecs(const std::vector<Vector4d>& r3_vec_hs) {
        if (r3_vec_hs.empty()) {
            throw std::invalid_argument("[RMUtils::Homos2R3Vecs() Error] The input list of homogeneous vectors is empty.");
        }
        std::vector<Vector3d> r3_vecs;
        for (const auto& r3_vec_h : r3_vec_hs) {
            Vector3d r3_vec = Homo2R3Vec(r3_vec_h);
            r3_vecs.push_back(r3_vec);
        }
        return r3_vecs; // {v1, v2, v3, ...}
    }

    static Vector3d ImgCoord2Homo(const Vector2d& img_coord, double s = 1.0) {
        Vector3d img_coord_h;
        img_coord_h << img_coord * s, s;
        return img_coord_h;
    }

    static std::vector<Vector3d> ImgCoords2Homos(const std::vector<Vector2d>& img_coords, double s = 1.0) {
        if (img_coords.empty()) {
            throw std::invalid_argument("[RMUtils::ImgCoords2Homos() Error] The input list of image coordinates is empty.");
        }
        std::vector<Vector3d> img_coords_h;
        for (const auto& img_coord : img_coords) {
            Vector3d img_coord_h = ImgCoord2Homo(img_coord, s);
            img_coords_h.push_back(img_coord_h);
        }
        return img_coords_h; // {img_coord_h_1, img_coord_h_2, img_coord_h_3, ...}
    }

    static Vector2d Homo2ImgCoord(const Vector3d& img_coord_h) {
        if (img_coord_h(2) == 0) {
            throw std::invalid_argument("[RMUtils::Homo2ImgCoord() Error] The homogeneous coordinate (last element) must not be zero.");
        }
        return img_coord_h.head<2>() / img_coord_h(2);
    }

    static std::vector<Vector2d> Homos2ImgCoords(const std::vector<Vector3d>& img_coords_hs) {
        if (img_coords_hs.empty()) {
            throw std::invalid_argument("[RMUtils::Homos2ImgCoords() Error] The input list of homogeneous image coordinates is empty.");
        }
        std::vector<Vector2d> img_coords;
        for (const auto& img_coord_h : img_coords_hs) {
            Vector2d img_coord = Homo2ImgCoord(img_coord_h);
            img_coords.push_back(img_coord);
        }
        return img_coords; // {img_coord_1, img_coord_2, img_coord_3, ...}
    }

    // SO(3) and so(3) functions (quaternions as main representation)
    // Quaternion operations
    static Quaterniond ConjQuat(const Quaterniond& quat) {
        return quat.conjugate();
    }

    static Quaterniond InvQuat(const Quaterniond& quat) {
        return quat.conjugate(); // Inverse of a unit quaternion is its conjugate
    }

    static Quaterniond TransformQuats(const std::vector<Quaterniond>& quats) {
        if (quats.empty()) {
            throw std::invalid_argument("[RMUtils::TransformQuats() Error] The input list of quaternions is empty.");
        }
        Quaterniond quat_accum = quats[0];
        for (size_t i = 1; i < quats.size(); ++i) {
            quat_accum *= quats[i];
        }
        return quat_accum;
    }

    // Exp and Log maps in SO(3)
    static Matrix3d R3Vec2so3Mat(const Vector3d& v) {
        Matrix3d so3Mat;
        so3Mat << 0, -v(2), v(1),
                  v(2), 0, -v(0),
                  -v(1), v(0), 0;
        return so3Mat;
    }

    static Vector3d so3Mat2R3Vec(const Matrix3d& so3Mat) {
        if (!NearZero((so3Mat + so3Mat.transpose()).norm())) {
            throw std::invalid_argument("[RMUtils::so3Mat2R3Vec() Error] The input matrix is not skew-symmetric.");
        }
        Vector3d v;
        v << so3Mat(2, 1), so3Mat(0, 2), so3Mat(1, 0);
        return v;
    }

    static Vector4d AxisAng3(const Vector3d& so3) {
        Vector4d v;
        v.head<3>() = so3.normalized();
        v(3) = so3.norm();
        return v; // {uhat_x, uhat_y, uhat_z, theta}
    }

    static Vector3d Quat2so3(const Quaterniond& quat) {
        // Normalize the quaternion to ensure it's a unit quaternion
        Quaterniond q = quat.normalized();
        double theta = 2 * ArcCos( ( q.w() > 0 ? q.w() : -q.w() ) , true);
        return (2 / Sinc(theta / 2)) * ( q.w() > 0 ? Vector3d(q.vec()) : Vector3d(-q.vec()) );
    }

    static Quaterniond so32Quat(const Vector3d& so3) {
        Vector4d axis_ang = AxisAng3(so3);
        double half_theta = axis_ang(3) / 2;
        Quaterniond q;
        q.w() = std::cos(half_theta);
        q.vec() = std::sin(half_theta) * axis_ang.head<3>();
        return q;  // q = cos(theta/2) + sin(theta/2) * uhat
    }

    static Matrix3d MatrixExp3(const Matrix3d& so3Mat) {
        Vector3d so3 = so3Mat2R3Vec(so3Mat);
        if (NearZero(so3.norm())) {
            return Matrix3d::Identity();
        } else {
            double theta = so3.norm();
            Vector3d omega = so3 / theta;
            Matrix3d omega_hat = R3Vec2so3Mat(omega);
            return Matrix3d::Identity() + std::sin(theta) * omega_hat + (1 - std::cos(theta)) * (omega_hat * omega_hat);
        }
    }

    static Matrix3d MatrixLog3(const Matrix3d& R) {
        double theta = std::acos((R.trace() - 1) / 2.0);
        if (NearZero(theta)) {
            return Matrix3d::Zero();
        } else {
            return (R - R.transpose()) * (theta / (2 * std::sin(theta)));
        }
    }

    // Rotation matrices
    static Matrix3d ThreeAxes2Rot(const Vector3d& x_1_2, const Vector3d& y_1_2, const Vector3d& z_1_2) {
        Matrix3d R_1_2;
        R_1_2.col(0) = x_1_2;
        R_1_2.col(1) = y_1_2;
        R_1_2.col(2) = z_1_2;
        return R_1_2;
    }

    static Matrix3d TransformRots(const std::vector<Matrix3d>& Rs) {
        if (Rs.empty()) {
            throw std::invalid_argument("[RMUtils::TransformRots() Error] Input vector of rotation matrices is empty.");
        }
        Matrix3d result = Rs[0];
        for (size_t i = 1; i < Rs.size(); ++i) {
            result = result * Rs[i];
        }
        return result;
    }

    static Matrix3d so32Rot(const Vector3d& so3) {
        return MatrixExp3(R3Vec2so3Mat(so3));
    }

    static Vector3d Rot2so3(const Matrix3d& R) {
        return so3Mat2R3Vec(MatrixLog3(R));
    }

    static Matrix3d Quat2Rot(const Quaterniond& quat) {
        // return quat.normalized().toRotationMatrix();
        return quat.toRotationMatrix();
    }

    static Quaterniond Rot2Quat(const Matrix3d& R) {
        Quaterniond q(R);
        // q.normalize();
        return q;
    }

    // ZYX Euler angles
    static Matrix3d Rotx(double thx, bool rad = true) {
        if (!rad) {
            thx *= d2r;
        }
        return Matrix3d(Eigen::AngleAxisd(thx, Vector3d::UnitX()));
    }

    static Matrix3d Roty(double thy, bool rad = true) {
        if (!rad) {
            thy *= d2r;
        }
        return Matrix3d(Eigen::AngleAxisd(thy, Vector3d::UnitY()));
    }

    static Matrix3d Rotz(double thz, bool rad = true) {
        if (!rad) {
            thz *= d2r;
        }
        return Matrix3d(Eigen::AngleAxisd(thz, Vector3d::UnitZ()));
    }

    static Matrix3d Rotxyz(const Vector3d& thxyz, bool rad = true) {
        Matrix3d Rx = Rotx(thxyz(0), rad);
        Matrix3d Ry = Roty(thxyz(1), rad);
        Matrix3d Rz = Rotz(thxyz(2), rad);
        return Rx * Ry * Rz;
    }

    static Matrix3d Rotzyx(const Vector3d& thzyx, bool rad = true) {
        Matrix3d Rz = Rotz(thzyx(0), rad);
        Matrix3d Ry = Roty(thzyx(1), rad);
        Matrix3d Rx = Rotx(thzyx(2), rad);
        return Rz * Ry * Rx;
    }

    static Vector3d Rot2zyxEuler(const Matrix3d& Rotzyx, bool rad = true) {
        double sy = -Rotzyx(2, 0);
        double cy = sqrt(Rotzyx(0, 0) * Rotzyx(0, 0) + Rotzyx(1, 0) * Rotzyx(1, 0));
        double thy = atan2(sy, cy);

        double sz = Rotzyx(1, 0) / cy;
        double cz = Rotzyx(0, 0) / cy;
        double thz = atan2(sz, cz);

        double sx = Rotzyx(2, 1) / cy;
        double cx = Rotzyx(2, 2) / cy;
        double thx = atan2(sx, cx);

        if (!rad) {
            thx *= r2d;
            thy *= r2d;
            thz *= r2d;
        }
        return Vector3d(thz, thy, thx);
    }

    static Quaterniond Quatx(double thx, bool rad = true) {
        if (!rad) {
            thx *= d2r;
        }
        return Quaterniond(Eigen::AngleAxisd(thx, Vector3d::UnitX()));
    }

    static Quaterniond Quaty(double thy, bool rad = true) {
        if (!rad) {
            thy *= d2r;
        }
        return Quaterniond(Eigen::AngleAxisd(thy, Vector3d::UnitY()));
    }

    static Quaterniond Quatz(double thz, bool rad = true) {
        if (!rad) {
            thz *= d2r;
        }
        return Quaterniond(Eigen::AngleAxisd(thz, Vector3d::UnitZ()));
    }

    static Quaterniond xyzEuler2Quat(const Vector3d& xyz_euler, bool rad = true) {
        double x_angle = rad ? xyz_euler(0) : xyz_euler(0) * d2r;
        double y_angle = rad ? xyz_euler(1) : xyz_euler(1) * d2r;
        double z_angle = rad ? xyz_euler(2) : xyz_euler(2) * d2r;

        Quaterniond qx(Eigen::AngleAxisd(x_angle, Vector3d::UnitX()));
        Quaterniond qy(Eigen::AngleAxisd(y_angle, Vector3d::UnitY()));
        Quaterniond qz(Eigen::AngleAxisd(z_angle, Vector3d::UnitZ()));

        Quaterniond q = qx * qy * qz;
        // q.normalize(); // Ensure the quaternion is normalized

        return q;
    }

    static Quaterniond zyxEuler2Quat(const Vector3d& zyx_euler, bool rad = true) {
        double z_angle = rad ? zyx_euler(0) : zyx_euler(0) * d2r;
        double y_angle = rad ? zyx_euler(1) : zyx_euler(1) * d2r;
        double x_angle = rad ? zyx_euler(2) : zyx_euler(2) * d2r;
        Quaterniond qz(Eigen::AngleAxisd(z_angle, Vector3d::UnitZ()));
        Quaterniond qy(Eigen::AngleAxisd(y_angle, Vector3d::UnitY()));
        Quaterniond qx(Eigen::AngleAxisd(x_angle, Vector3d::UnitX()));
        Quaterniond q = qz * qy * qx;
        return q;
    }

    static Vector3d Quat2zyxEuler(const Quaterniond& quat, bool rad = true) {
        // Matrix3d R = quat.normalized().toRotationMatrix();
        Matrix3d R = quat.toRotationMatrix();
        return Rot2zyxEuler(R, rad);
    }

    // SE(3) and se(3) functions
    // Exp and Log maps in SE(3)
    static Matrix4d R6Vec2se3Mat(const Vector6d& V) {
        // Linear and angular parts
        Vector3d linear = V.head<3>();
        Vector3d ang = V.tail<3>();
        Matrix4d M = Matrix4d::Zero();
        M.block<3, 3>(0, 0) = R3Vec2so3Mat(ang);
        M.block<3, 1>(0, 3) = linear;
        // Last row is already zeros
        return M;
    }

    static Vector6d se3Mat2R6Vec(const Matrix4d& se3Mat) {
        Vector6d V;
        V.head<3>() = se3Mat.block<3, 1>(0, 3);
        V.tail<3>() << se3Mat(2, 1), se3Mat(0, 2), se3Mat(1, 0);
        return V; // {vx, vy, vz, wx, wy, wz}
    }

    static Vector7d AxisAng6(const Vector6d& se3) {
        Vector7d v_ret;
        double theta = se3.tail<3>().norm(); // Angular part
        if (NearZero(theta))
            theta = se3.head<3>().norm(); // Linear part
        v_ret.head<6>() = se3 / theta;
        v_ret(6) = theta;
        return v_ret; // {v/theta, theta}
    }

    static Matrix4d MatrixExp6(const Matrix4d& se3Mat) {
        // Extract the angular velocity vector from the transformation matrix
        Matrix3d se3Mat_cut = se3Mat.block<3, 3>(0, 0);
        Vector3d so3 = so3Mat2R3Vec(se3Mat_cut);
        Matrix4d m_ret = Matrix4d::Identity();
        // If negligible rotation
        if (NearZero(so3.norm())) {
            m_ret.block<3, 1>(0, 3) = se3Mat.block<3, 1>(0, 3);
            return m_ret;
        } else {
            double theta = so3.norm();
            Vector3d omega = so3 / theta;
            Matrix3d omega_hat = R3Vec2so3Mat(omega);
            Matrix3d R = Matrix3d::Identity() + std::sin(theta) * omega_hat + (1 - std::cos(theta)) * omega_hat * omega_hat;
            Matrix3d V = Matrix3d::Identity() * theta + (1 - std::cos(theta)) * omega_hat + (theta - std::sin(theta)) * omega_hat * omega_hat;
            Vector3d linear = se3Mat.block<3, 1>(0, 3);
            Vector3d p = V * (linear / theta);
            m_ret.block<3, 3>(0, 0) = R;
            m_ret.block<3, 1>(0, 3) = p;
            return m_ret;
        }
    }

    static Matrix4d MatrixLog6(const Matrix4d& T) {
        Matrix4d m_ret = Matrix4d::Zero();
        PosRot pos_rot = TMat2PosRot(T);
        Matrix3d R = pos_rot.rot;
        Vector3d p = pos_rot.pos;
        Matrix3d omega_hat = MatrixLog3(R);
        if (NearZero(omega_hat.norm())) {
            m_ret.block<3, 1>(0, 3) = p;
        } else {
            double theta = std::acos((R.trace() - 1) / 2.0);
            // Removed unused variable 'omega_hat_normalized'
            Matrix3d G_inv = Matrix3d::Identity() - 0.5 * omega_hat +
                (1 / (theta * theta) - (1 + std::cos(theta)) / (2 * theta * std::sin(theta))) * omega_hat * omega_hat;
            Vector3d v = G_inv * p;
            m_ret.block<3, 3>(0, 0) = omega_hat;
            m_ret.block<3, 1>(0, 3) = v;
        }
        return m_ret;
    }

    // Core conversions (quaternions to others)
    // pos_quat <-> r6_pose
    static Vector6d PosQuat2R6Pose(const PosQuat& pos_quat_1_2) {
        Vector6d pose_1_2;
        pose_1_2.head<3>() = pos_quat_1_2.pos;
        pose_1_2.tail<3>() = Quat2zyxEuler(pos_quat_1_2.quat, true).reverse();
        return pose_1_2;
    }

    static PosQuat R6Pose2PosQuat(const Vector6d& pose_1_2) {
        PosQuat pos_quat_1_2;
        pos_quat_1_2.pos = pose_1_2.head<3>();
        pos_quat_1_2.quat = zyxEuler2Quat(pose_1_2.tail<3>().reverse(), true);
        return pos_quat_1_2;
    }

    // pos_quat <-> pos_so3
    static Vector6d PosQuat2Posso3(const PosQuat& pos_quat_1_2) {
        Vector6d pos_so3_1_2;
        pos_so3_1_2.head<3>() = pos_quat_1_2.pos;
        pos_so3_1_2.tail<3>() = Quat2so3(pos_quat_1_2.quat);
        return pos_so3_1_2;
    }

    static PosQuat Posso32PosQuat(const Vector6d& pos_so3_1_2) {
        PosQuat pos_quat_1_2;
        pos_quat_1_2.pos = pos_so3_1_2.head<3>();
        pos_quat_1_2.quat = so32Quat(pos_so3_1_2.tail<3>());
        return pos_quat_1_2;
    }

    // pos_quat <-> transformation matrix
    static Matrix4d PosQuat2TMat(const PosQuat& pos_quat_1_2) {
        Matrix4d T_1_2 = Matrix4d::Identity();
        T_1_2.topLeftCorner<3, 3>() = Quat2Rot(pos_quat_1_2.quat);
        T_1_2.topRightCorner<3, 1>() = pos_quat_1_2.pos;
        return T_1_2;
    }

    static PosQuat TMat2PosQuat(const Matrix4d& T_1_2) {
        PosQuat pos_quat_1_2;
        pos_quat_1_2.pos = T_1_2.topRightCorner<3, 1>();
        pos_quat_1_2.quat = Rot2Quat(T_1_2.topLeftCorner<3, 3>());
        return pos_quat_1_2;
    }

    // Other conversions
    // rot_pos <-> transformation matrix
    static Matrix4d PosRot2TMat(const PosRot& pos_rot_1_2) {
        Matrix4d T_1_2 = Matrix4d::Identity();
        T_1_2.topLeftCorner<3, 3>() = pos_rot_1_2.rot;
        T_1_2.topRightCorner<3, 1>() = pos_rot_1_2.pos;
        return T_1_2;
    }

    static PosRot TMat2PosRot(const Matrix4d& T_1_2) {
        PosRot pos_rot_1_2;
        pos_rot_1_2.pos = T_1_2.topRightCorner<3, 1>();
        pos_rot_1_2.rot = T_1_2.topLeftCorner<3, 3>();
        return pos_rot_1_2;
    }

    // r6_pose <-> transformation matrix
    static Matrix4d R6Pose2TMat(const Vector6d& pose_1_2) {
        return PosQuat2TMat(R6Pose2PosQuat(pose_1_2));
    }

    static Vector6d TMat2R6Pose(const Matrix4d& T_1_2) {
        return PosQuat2R6Pose(TMat2PosQuat(T_1_2));
    }

    // r6_pose <-> rot_pos
    static PosRot R6Pose2PosRot(const Vector6d& pose_1_2) {
        PosQuat pos_quat_1_2 = R6Pose2PosQuat(pose_1_2);
        PosRot pos_rot_1_2;
        pos_rot_1_2.pos = pos_quat_1_2.pos;
        pos_rot_1_2.rot = Quat2Rot(pos_quat_1_2.quat);
        return pos_rot_1_2;
    }

    static Vector6d PosRot2R6Pose(const PosRot pos_rot_1_2) {
        return PosQuat2R6Pose(TMat2PosQuat(PosRot2TMat(pos_rot_1_2)));
    }

    // transformation matrix <-> pos_so3
    static Vector6d TMat2Posso3(const Matrix4d& T_1_2) {
        return PosQuat2Posso3(TMat2PosQuat(T_1_2));
    }

    static Matrix4d Posso32TMat(const Vector6d& pos_so3_1_2) {
        return PosQuat2TMat(Posso32PosQuat(pos_so3_1_2));
    }

    // Inverse transformations
    static PosQuat InvPosQuat(const PosQuat& pos_quat_1_2) {
        PosQuat pos_quat_2_1;
        pos_quat_2_1.pos = -(pos_quat_1_2.quat.conjugate() * pos_quat_1_2.pos); // -q_2_1 (p_1_2) q_2_1*
        pos_quat_2_1.quat = pos_quat_1_2.quat.conjugate();
        return pos_quat_2_1;
    }

    static Vector6d InvR6Pose(const Vector6d& pose_1_2) {
        return PosQuat2R6Pose(InvPosQuat(R6Pose2PosQuat(pose_1_2)));
    }

    // Transform poses and relative poses
    // pos_quats
    static PosQuat TransformPosQuats(const std::vector<PosQuat>& pos_quats) {
        if (pos_quats.empty()) {
            throw std::invalid_argument("[RMUtils::TransformPosQuats() Error] The input list of pos_quats is empty.");
        }
        PosQuat pos_quat_accum = pos_quats[0];
        for (size_t i = 1; i < pos_quats.size(); ++i) {
            const PosQuat& pos_quat_i = pos_quats[i];
            // Rotate and translate the position
            pos_quat_accum.pos = pos_quat_accum.quat * pos_quat_i.pos + pos_quat_accum.pos; // p_1_3 = q_1_2 (p_2_3) q_1_2* + p_1_2
            // Compute the new quaternion
            pos_quat_accum.quat *= pos_quat_i.quat; // q_1_3 = q_1_2 * q_2_3
        }
        return pos_quat_accum;
    }

    static PosQuat PosQuats2RelativePosQuat(const PosQuat& pos_quat_b_1, const PosQuat& pos_quat_b_2) {
        return TransformPosQuats( {InvPosQuat(pos_quat_b_1), pos_quat_b_2} ); // pos_quat_1_2
    }    

    // transformation matrix matrices
    static Matrix4d TransformTMats(const std::vector<Matrix4d>& Ts) {
        if (Ts.empty()) {
            throw std::invalid_argument("[RMUtils::TransformTMats() Error] Input vector of T matrices is empty.");
        }
        Matrix4d result = Ts[0];
        for (size_t i = 1; i < Ts.size(); ++i) {
            result = result * Ts[i];
        }
        return result;
    }

    // 6D poses
    static Vector6d TransformR6Poses(const std::vector<Vector6d>& poses) {
        if (poses.empty()) {
            throw std::invalid_argument("[RMUtils::TransformR6Poses() Error] The input list of poses is empty.");
        }
        PosQuat pos_quat_accum = R6Pose2PosQuat(poses[0]);
        for (std::size_t i = 1; i < poses.size(); ++i) {
            PosQuat pos_quat_i = R6Pose2PosQuat(poses[i]);
            pos_quat_accum = TransformPosQuats({pos_quat_accum, pos_quat_i});
        }
        Vector6d pose_accum = PosQuat2R6Pose(pos_quat_accum);
        return pose_accum;
    }

    static Vector6d R6Poses2RelativeR6Pose(const Vector6d& pose_b_1, const Vector6d& pose_b_2) {
        return TransformR6Poses( {InvR6Pose(pose_b_1), pose_b_2} ); // pose_1_2
    }

    /* Robot kinematics, motion Planning and control */
    
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
        t1.quat = RMUtils::Quatx(alpha);

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
        t4.quat = RMUtils::Quatz(theta);

        // Combine: T = R_x(alpha) * Trans_x(a) * Trans_z(d) * R_z(theta)
        return RMUtils::TransformPosQuats({t1, t2, t3, t4});
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
     * @param table        D-H table containing alpha, a, d, and default theta for each joint
     * @param theta_list   Joint offsets [rad] to add to the default theta values; size must match number of joints
     * @return             Combined end-effector pose as PosQuat
     */
    static PosQuat FKDH(const DHTable& table, const VectorXd& theta_list) {
        // Number of joints
        size_t n = table.joints.size();
        if (static_cast<size_t>(theta_list.size()) != n) {
            throw std::invalid_argument("[RMUtils::FKDH() Error] theta_list size must match number of joints in D-H table.");
        }
        // Compute each joint's D-H transform using default theta + offset
        std::vector<PosQuat> transforms;
        transforms.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            const auto& dh = table.joints[i];
            double theta = dh.theta + theta_list(static_cast<int>(i)); // D-H offset angle + joint angle
            transforms.push_back(DHTransform(dh.alpha, dh.a, dh.d, theta));
        }
        // Chain all transforms to get end-effector pose
        return TransformPosQuats(transforms);
    }

    /**
     * @brief Compute forward kinematics by D-H at zero configuration and generate ScrewList
     * @param table D-H table containing joints and default theta values
     * @return      ScrewList with screw axes in end-effector frame and home pose
     */
    static ScrewList ScrewListFromDH(const DHTable& table) {
        // number of joints
        size_t n = table.joints.size();
        // zero theta offsets
        VectorXd zero_theta_offsets = VectorXd::Zero(n);
        // compute zero configuration pose via FKDH
        PosQuat zero_config_pose = FKDH(table, zero_theta_offsets);
        Matrix4d T_b_e_zero = PosQuat2TMat(zero_config_pose);

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
            PosQuat pq_i = TransformPosQuats(partial);
            T_b_ji_list.push_back(PosQuat2TMat(pq_i));
        }

        // build screw_list in end-effector frame
        Eigen::MatrixXd screw_list(6, n);
        for (size_t i = 0; i < n; ++i) {
            // pose of joint frame w.r.t. end-effector
            Eigen::Matrix4d T_e_ji = T_b_e_zero.inverse() * T_b_ji_list[i];
            Eigen::Vector3d u_hat = TMat2PosRot(T_e_ji).rot.col(2); // axis of rotation: z-axis of joint frame w.r.t. ee
            Eigen::Vector3d r = -TMat2PosRot(T_e_ji).pos; // rotation radius vector: (negative) position of joint frame w.r.t. ee
            Eigen::Vector3d v = u_hat.cross(r); // linear velocity: v = u_hat x r

            Vector6d S;
            S.head<3>() = v;
            S.tail<3>() = u_hat;
            screw_list.col(i) = S;
        }

        return ScrewList(screw_list, zero_config_pose);
    }


    /**
     * @brief PoE forward kinematics: T = M * exp([S_e,1]θ1) * ... * exp([S_e,n]θn),
     *        where S_e,i is the i'th screw axis in end-effector frame.
     * @param screws     ScrewList containing:
     *                     - screw_list: 6×n matrix, each column a end-effector frame screw axis
     *                     - M:         zero configuration pose of the end-effector
     * @param theta_list Joint angle offsets [rad], size n
     * @return           End-effector pose as PosQuat
     */
    static PosQuat FKPoE(const ScrewList& screws, const VectorXd& theta_list) {
        int n = static_cast<int>(theta_list.size());
        if (screws.screw_list.cols() != n) {
            throw std::invalid_argument("[RMUtils::FKPoE() Error] theta_list length must match number of screws in screw list.");
        }
        // start from home pose
        Eigen::Matrix4d T = PosQuat2TMat(screws.M);
        // apply each exp([Si] θi)
        for (int i = 0; i < n; ++i) {
            Vector6d screw6 = screws.screw_list.col(i) * theta_list(i);
            Eigen::Matrix4d exp_se3 = MatrixExp6(R6Vec2se3Mat(screw6));
            T = T * exp_se3;
        }
        return TMat2PosQuat(T);
    }



    // Velocity adjoint maps
    static Matrix6d Adjoint(const Matrix3d& R, const Vector3d& p) {
        Matrix6d adj = Matrix6d::Identity();
        Matrix3d p_skew = R3Vec2so3Mat(p);
        adj.topLeftCorner<3, 3>() = R;
        adj.topRightCorner<3, 3>() = p_skew * R;
        adj.bottomRightCorner<3, 3>() = R;
        return adj;
    }

    static Vector6d AdjointE2B(const Matrix3d& R_b_e, const Vector3d& p_offset, const Vector6d& twist_e_e) {
        Matrix6d adj_e2b = Adjoint(R_b_e, p_offset);
        return adj_e2b * twist_e_e;
    }

    static Vector6d AdjointB2E(const Matrix3d& R_b_e, const Vector3d& p_offset, const Vector6d& twist_b_e) {
        Matrix6d adj_b2e = Adjoint(R_b_e, p_offset).inverse();
        return adj_b2e * twist_b_e;
    }

    /* Pose preprocessing */
    static PosQuat PosQuatOutlierRemoval(const PosQuat& current_pos_quat, double std_thresh, std::deque<Vector6d>& buffer, std::size_t window_size) {
        // Convert pos_quat to pos_so3
        Vector6d current_pos_so3 = PosQuat2Posso3(current_pos_quat);
        // Check if buffer has enough data
        if (buffer.size() < window_size) {
            // Not enough data, accept current_pos_quat
            buffer.push_back(current_pos_so3);
            return current_pos_quat;
        } else {
            // Compute mean and std for positions and so3
            Vector6d mean_buffer = MeanBuffer(buffer);
            Vector6d std_buffer = StdBuffer(buffer);
            double dev_pos_norm = (current_pos_so3.head<3>() - mean_buffer.head<3>()).norm();
            double dev_so3_norm = (current_pos_so3.tail<3>() - mean_buffer.tail<3>()).norm();
            double std_pos_norm = std_buffer.head<3>().norm();
            double std_so3_norm = std_buffer.tail<3>().norm();

            // Check if deviation exceeds threshold times standard deviation
            bool is_outlier = false;
            if (std_pos_norm > 0 && dev_pos_norm > std_thresh * std_pos_norm) {
                is_outlier = true;
                std::cout << "dev_pos_norm = " << dev_pos_norm << ", std_pos_norm = " << std_pos_norm << std::endl;
            } else if (std_so3_norm > 0 && dev_so3_norm > std_thresh * std_so3_norm) {
                is_outlier = true;
                std::cout << "dev_so3_norm = " << dev_so3_norm << ", std_so3_norm = " << std_so3_norm << std::endl;
            }

            if (is_outlier) {
                // Outlier detected, return previous accepted value
                std::cout << "Outlier detected, rejecting current value." << std::endl;
                // Return previous value converted back to pos_quat
                Vector6d previous_pos_so3 = buffer.back();
                return Posso32PosQuat(previous_pos_so3);
            } else {
                // Accept current value, update buffer
                buffer.push_back(current_pos_so3);
                // Keep buffer size within window_size
                if (buffer.size() > window_size) {
                    buffer.pop_front();
                }
                return current_pos_quat;
            }
        }
    }

    // PosQuat moving averaging. First convert into so(3), mavg, and convert back to PosQuat
    static PosQuat PosQuatMAvg(const PosQuat& current_pos_quat, std::deque<Vector6d>& buffer, std::size_t window_size) {
        // Convert pos_quat to pos_so3
        Vector6d current_pos_so3 = PosQuat2Posso3(current_pos_quat);
        // Compute MAvg
        Vector6d mavg_pos_so3 = MAvg(current_pos_so3, buffer, window_size);
        return Posso32PosQuat(mavg_pos_so3); // mavg_pos_quat
    }

    /* Motion mapping */
    static Vector6d AxisDecoupling(const Vector6d& joy_input) {
        double max_val = 0.68;
        Vector6d thres_percent;
        thres_percent << 0.45, 0.45, 0.55, 0.90, 0.90, 0.90; // To be tuned according to needs

        Vector6d out_thresh = Vector6d::Zero();
        Vector6d joy_dec = Vector6d::Zero();

        double max_temp = 0;
        int max_id = -1;
        for (int i = 0; i < 6; i++)
        {
            double sig = joy_input(i);
            // Check threshold
            if (std::abs(sig) >= (thres_percent(i) * max_val))
            {
                out_thresh(i) = sig > 0 ? 1 : -1; // 1 if positive, -1 if negative
                if ((std::abs(sig) > max_temp) && !((i == 2 && max_id != -1) || (max_id == 2)))
                {
                    max_temp = std::abs(sig);
                    max_id = i;
                }
            }
        }

        if (max_id != -1)
        {
            joy_dec(max_id) = out_thresh(max_id);
        }
        return joy_dec;
    }

    static Vector6d VelMapping(const Vector6d& joy_input, const Matrix6d& vel_scale) {
        return vel_scale * joy_input; // vx, vy, vz, wx, wy, wz [m/s, rad/s]
    }

    /* Motion planning */
    // S-curve velocity smoothing
    static Vector6d SCurve(const Vector6d& twist_cmd, const Vector6d& twist_cmd_prev, double lambda, double t, double T)
    {   
        // twist_cmd: cmd
        // twist_cmd_prev: start value
        // lambda: steepness of the curve (determines max. acc.)
        // t: time elapsed
        // T: total time for the curve to reach the final value
        if (lambda <= 0 || T <= 0) {
            throw std::invalid_argument("[RMUtils::SCurve() Error] The lambda and T must be positive.");
        }
        return twist_cmd_prev + (twist_cmd - twist_cmd_prev) / (1 + std::exp(-lambda * (t - T / 2.0)));
    }

    // S-curve for scalar
    static double SCurve(const double& cmd, const double& cmd_prev, double lambda, double t, double T)
    {   
        // cmd: cmd
        // cmd_prev: start value
        // lambda: steepness of the curve (determines max. acc.)
        // t: time elapsed
        // T: total time for the curve to reach the final value
        if (lambda <= 0 || T <= 0) {
            throw std::invalid_argument("[RMUtils::SCurve() Error] The lambda and T must be positive.");
        }
        return cmd_prev + (cmd - cmd_prev) / (1 + std::exp(-lambda * (t - T / 2.0)));
    }

    // Screw motion path generation (only waypoints)
    // 3D screw motion
    // Position: linear interpolation
    // Orientation: so(3) interpolation
    static std::vector<PosQuat> ScrewMotion3DPath(const PosQuat& pos_quat_b_e, const PosQuat& pos_quat_e_e_cmd, int N) {
        std::vector<PosQuat> waypoints; // Output waypoints
        // Convert pos_quat_e_e_cmd to pos_so3_e_e_cmd
        Vector6d pos_so3_e_e_cmd = PosQuat2Posso3(pos_quat_e_e_cmd);
        // Generate waypoints
        for (int i = 0; i < N; ++i)
        {
            double alpha = static_cast<double>(i + 1) / N;
            // Intermediate pos_so3
            Vector6d pos_so3_e_e_cmd_i = alpha * pos_so3_e_e_cmd;
            // Compute pos_so3 back to pos_quat
            PosQuat pos_quat_e_e_cmd_i = Posso32PosQuat(pos_so3_e_e_cmd_i);
            // Transform to base frame
            PosQuat pos_quat_b_e_d_i = TransformPosQuats({pos_quat_b_e, pos_quat_e_e_cmd_i});
            // Append to waypoints
            waypoints.push_back(pos_quat_b_e_d_i);
        }
        return waypoints;
    }

    // Screw motion 3D trajectory generation (waypoints and timestamps)
    static std::pair<std::vector<PosQuat>, std::vector<double>> ScrewMotion3DTraj(const PosQuat& pos_quat_b_e, const PosQuat& pos_quat_e_e_cmd, int N, double T) {
        std::vector<PosQuat> waypoints; // Output waypoints
        std::vector<double> timestamps; // Output timestamps
        // Convert pos_quat_e_e_cmd to pos_so3_e_e_cmd
        Vector6d pos_so3_e_e_cmd = PosQuat2Posso3(pos_quat_e_e_cmd);
        // Generate waypoints
        for (int i = 0; i < N; ++i)
        {
            double alpha = static_cast<double>(i + 1) / N;
            // Intermediate pos_so3
            Vector6d pos_so3_e_e_cmd_i = alpha * pos_so3_e_e_cmd;
            // Compute pos_so3 back to pos_quat
            PosQuat pos_quat_e_e_cmd_i = Posso32PosQuat(pos_so3_e_e_cmd_i);
            // Transform to base frame
            PosQuat pos_quat_b_e_d_i = TransformPosQuats({pos_quat_b_e, pos_quat_e_e_cmd_i});
            // Append to waypoints
            waypoints.push_back(pos_quat_b_e_d_i);
            // Append timestamps
            timestamps.push_back(alpha * T);
        }
        return std::make_pair(waypoints, timestamps);
    }

    // 6D screw motion: interpolation in se(3)
    static std::vector<PosQuat> ScrewMotion6DPath(const PosQuat& pos_quat_b_e, const PosQuat& pos_quat_e_e_cmd, int N) {
        std::vector<PosQuat> waypoints; // Output waypoints
         // Convert pos_quat_e_e_cmd to se3_e_e_cmd
        Vector6d se3_e_e_cmd = se3Mat2R6Vec( MatrixLog6( PosQuat2TMat(pos_quat_e_e_cmd) ) );
        // Generate waypoints
        for (int i = 0; i < N; ++i)
        {
            double alpha = static_cast<double>(i + 1) / N;
            // Intermediate se3
            Vector6d se3_e_e_cmd_i = alpha * se3_e_e_cmd;
            // Compute se3 back to pos_quat
            PosQuat pos_quat_e_e_cmd_i = TMat2PosQuat( MatrixExp6( R6Vec2se3Mat(se3_e_e_cmd_i) ) );
            // Transform to base frame
            PosQuat pos_quat_b_e_d_i = TransformPosQuats({pos_quat_b_e, pos_quat_e_e_cmd_i});
            // Append to waypoints
            waypoints.push_back(pos_quat_b_e_d_i);
        }
        return waypoints;
    }

    // Screw motion 6D trajectory generation (waypoints and timestamps)
    static std::pair<std::vector<PosQuat>, std::vector<double>> ScrewMotion6DTraj(const PosQuat& pos_quat_b_e, const PosQuat& pos_quat_e_e_cmd, int N, double T) {
        std::vector<PosQuat> waypoints; // Output waypoints
        std::vector<double> timestamps; // Output timestamps
        // Convert pos_quat_e_e_cmd to se3_e_e_cmd
        Vector6d se3_e_e_cmd = se3Mat2R6Vec( MatrixLog6( PosQuat2TMat(pos_quat_e_e_cmd) ) );
        // Generate waypoints
        for (int i = 0; i < N; ++i)
        {
            double alpha = static_cast<double>(i + 1) / N;
            // Intermediate se3
            Vector6d se3_e_e_cmd_i = alpha * se3_e_e_cmd;
            // Compute se3 back to pos_quat
            PosQuat pos_quat_e_e_cmd_i = TMat2PosQuat( MatrixExp6( R6Vec2se3Mat(se3_e_e_cmd_i) ) );
            // Transform to base frame
            PosQuat pos_quat_b_e_d_i = TransformPosQuats({pos_quat_b_e, pos_quat_e_e_cmd_i});
            // Append to waypoints
            waypoints.push_back(pos_quat_b_e_d_i);
            // Append timestamps
            timestamps.push_back(alpha * T);
        }
        return std::make_pair(waypoints, timestamps);
    }

    /* Robot controller functions */
    static std::pair<Vector6d, bool> ErrorThreshold(const Vector2d& error_norm_mavg, const Vector2d& error_norm_thresh, Vector6d twist_cmd) {
        bool target_reached = false;
        if ((error_norm_mavg.array() <= error_norm_thresh.array()).all()) {
            target_reached = true;
            twist_cmd.setZero();  // Set twist_cmd to zero vector
        }
        return std::make_pair(twist_cmd, target_reached);
    }

    // Cartesian kinematic control
    static Vector6d KpPosso3(const Vector6d& pos_so3_m_cmd, const Matrix6d& kp_pos_so3, bool target_reached) {     
        Vector6d twist_cmd = kp_pos_so3 * pos_so3_m_cmd;
        if (target_reached) {
            twist_cmd.setZero();
        }
        return twist_cmd;
    }

};

#endif // ROBOT_MATH_UTILS_HPP
