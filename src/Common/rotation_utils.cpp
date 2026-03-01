/**
 * @file rotation_utils.cpp
 * @brief Implementation of rotation utilities for photogrammetric pose representation
 *
 * See rotation_utils.h for detailed documentation.
 * See doc/rotation/ROTATION_STANDARDS.md for theoretical background.
 */

#include "rotation_utils.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iomanip>

#include <glog/logging.h>

namespace insight {

// ============================================================================
// GIMBAL LOCK DETECTION
// ============================================================================

bool IsGimbalLockRisk(double phi_rad, double threshold) {
    const double PI_2 = M_PI / 2.0;
    
    // Check if phi is close to +π/2 or -π/2
    double distance_to_positive = std::abs(phi_rad - PI_2);
    double distance_to_negative = std::abs(phi_rad + PI_2);
    
    double min_distance = std::min(distance_to_positive, distance_to_negative);
    
    return min_distance < threshold;
}

double GetGimbalLockMagnitude(double phi_rad) {
    const double PI_2 = M_PI / 2.0;
    const double THRESHOLD = 0.1;  // Safe zone radius (≈5.7°)
    
    double distance_to_positive = std::abs(phi_rad - PI_2);
    double distance_to_negative = std::abs(phi_rad + PI_2);
    
    double min_distance = std::min(distance_to_positive, distance_to_negative);
    
    if (min_distance > THRESHOLD) {
        return 0.0;  // Safe
    }
    
    // Inverse: closer to singularity = higher magnitude
    // When distance = 0, magnitude should be ~1.0 or higher
    // When distance = THRESHOLD, magnitude should be ~0.0
    return 1.0 - (min_distance / THRESHOLD);
}

// ============================================================================
// CONVERSION: OmegaPhiKappa ↔ Rotation Matrix
// ============================================================================

Mat3 OPK_to_RotationMatrix(double omega, double phi, double kappa,
                           bool allow_gimbal_lock) {
    if (!allow_gimbal_lock && IsGimbalLockRisk(phi, 0.1)) {
        std::ostringstream oss;
        oss << "Gimbal lock detected! φ = " << std::fixed << std::setprecision(6)
            << phi << " rad (" << (phi * 180.0 / M_PI) << "°). "
            << "Consider using quaternions instead.";
        LOG(WARNING) << oss.str();
    }
    
    // Compute rotation matrices for each axis
    // R_x(ω) - rotation around X axis
    Mat3 Rx = Eigen::AngleAxisd(omega, Eigen::Vector3d::UnitX()).toRotationMatrix();
    
    // R_y(φ) - rotation around Y axis
    Mat3 Ry = Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitY()).toRotationMatrix();
    
    // R_z(κ) - rotation around Z axis
    Mat3 Rz = Eigen::AngleAxisd(kappa, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
    // Combined: R = R_z(κ) · R_y(φ) · R_x(ω)  [Z-Y-X extrinsic, fixed axes]
    // This represents: World → Camera transformation
    Mat3 R = Rz * Ry * Rx;
    
    return R;
}

void RotationMatrix_to_OPK(const Mat3& R,
                           double& omega,
                           double& phi,
                           double& kappa) {
    if (!IsValidRotationMatrix(R)) {
        std::ostringstream oss;
        oss << "Input is not a valid rotation matrix.\n"
            << "det(R) = " << R.determinant() << " (should be 1.0)\n"
            << "(R·R^T - I).norm() = " << (R * R.transpose() - Mat3::Identity()).norm()
            << " (should be ~0)";
        throw std::invalid_argument(oss.str());
    }
    
    // Extract OmegaPhiKappa from rotation matrix
    // For Z-Y-X extrinsic (R = R_z(κ) · R_y(φ) · R_x(ω)):
    // 
    // R = | cy·cz                  -cy·sz                   sy       |
    //     | sx·sy·cz + cx·sz        -sx·sy·sz + cx·cz    -sx·cy    |
    //     | -cx·sy·cz + sx·sz       cx·sy·sz + sx·cz      cx·cy    |
    //
    // where cx=cos(ω), sx=sin(ω), cy=cos(φ), sy=sin(φ), cz=cos(κ), sz=sin(κ)
    
    const double PI_2 = M_PI / 2.0;
    const double EPSILON = 1e-10;
    
    // From R(0,2) = sin(φ)
    phi = std::asin(R(0, 2));
    
    // Check if we're near gimbal lock
    double cos_phi = std::cos(phi);
    
    if (std::abs(cos_phi) < EPSILON) {
        // Gimbal lock case: φ = ±π/2
        // In this case, we can only determine ω + κ or ω - κ
        // Set ω = 0 and solve for κ
        
        omega = 0.0;
        
        // Use R(1,1) and R(1,0) to extract κ
        // At gimbal lock: R(1,1) = cos(0)*cos(κ) = cos(κ)
        //                R(1,0) = sin(0)*sin(κ) = 0 (when ω=0)
        // So we use: R(2,0) and R(2,1) instead
        kappa = std::atan2(-R(2, 0), R(2, 1));
        
        LOG(WARNING) << "Gimbal lock detected during extraction: φ = " << phi
                     << " rad (" << (phi * 180.0 / M_PI) << "°)";
    } else {
        // Normal case: extract ω from R(1,2) and R(2,2)
        // R(1,2) = -sin(ω)*cos(φ)
        // R(2,2) = cos(ω)*cos(φ)
        omega = std::atan2(-R(1, 2), R(2, 2));
        
        // Extract κ from R(0,0) and R(0,1)
        // R(0,0) = cos(φ)*cos(κ)
        // R(0,1) = -cos(φ)*sin(κ)
        kappa = std::atan2(-R(0, 1), R(0, 0));
    }
}

// ============================================================================
// CONVERSION: OmegaPhiKappa ↔ Quaternion
// ============================================================================

Quaternion OPK_to_Quaternion(double omega, double phi, double kappa) {
    // Method 1: Convert OPK to rotation matrix, then matrix to quaternion
    Mat3 R = OPK_to_RotationMatrix(omega, phi, kappa, true);  // Allow gimbal lock
    Quaternion q(R);
    
    return q.normalized();  // Ensure normalization
}

void Quaternion_to_OPK(const Quaternion& q,
                       double& omega,
                       double& phi,
                       double& kappa) {
    if (!IsValidQuaternion(q)) {
        std::ostringstream oss;
        oss << "Input quaternion is not normalized. Norm = " << q.norm();
        throw std::invalid_argument(oss.str());
    }
    
    // Convert quaternion to rotation matrix
    Mat3 R = q.toRotationMatrix();
    
    // Convert matrix to OPK angles
    RotationMatrix_to_OPK(R, omega, phi, kappa);
}

// ============================================================================
// CONVERSION: OmegaPhiKappa ↔ YawPitchRoll
// ============================================================================

void OPK_to_YPR(double opk_omega, double opk_phi, double opk_kappa,
                double& ypr_yaw, double& ypr_pitch, double& ypr_roll) {
    // Step 1: Convert OPK to rotation matrix
    // This represents World → Camera transformation
    Mat3 R_opk = OPK_to_RotationMatrix(opk_omega, opk_phi, opk_kappa, true);
    
    // Step 2: Account for coordinate frame difference
    // Photogrammetry camera frame: {X-right, Y-down, Z-forward}
    // Navigation body frame: {X-forward, Y-right, Z-down}
    //
    // Transformation matrix from photogrammetry to navigation:
    // The coordinate frame mapping:
    // Camera X (right) → Nav Y (right)
    // Camera Y (down)  → Nav Z (down)
    // Camera Z (forward) → Nav X (forward)
    //
    // So: R_nav = P · R_opk · P^T
    // where P maps photogrammetry axes to navigation axes
    
    Mat3 P;
    P << 0,  0,  1,    // Nav X (forward) = Camera Z (forward)
         1,  0,  0,    // Nav Y (right)   = Camera X (right)
         0,  1,  0;    // Nav Z (down)    = Camera Y (down)
    
    Mat3 R_nav = P * R_opk * P.transpose();
    
    // Step 3: Extract YPR from the navigation frame rotation matrix
    // For Z-Y-X intrinsic (body-fixed axes):
    // The rotation matrix components are similar to OPK but interpretation differs
    
    // For now, extract using same method as OPK (since underlying matrix is Z-Y-X)
    RotationMatrix_to_OPK(R_nav, ypr_roll, ypr_pitch, ypr_yaw);
}

void YPR_to_OPK(double ypr_yaw, double ypr_pitch, double ypr_roll,
                double& opk_omega, double& opk_phi, double& opk_kappa) {
    // Step 1: Convert YPR to rotation matrix (in navigation frame)
    Mat3 R_nav = OPK_to_RotationMatrix(ypr_roll, ypr_pitch, ypr_yaw, true);
    
    // Step 2: Apply inverse frame transformation
    Mat3 P;
    P << 0,  0,  1,
         1,  0,  0,
         0,  1,  0;
    
    Mat3 R_opk = P.transpose() * R_nav * P;
    
    // Step 3: Extract OPK from the transformed matrix
    RotationMatrix_to_OPK(R_opk, opk_omega, opk_phi, opk_kappa);
}

// ============================================================================
// VALIDATION
// ============================================================================

bool IsValidRotationMatrix(const Mat3& R, double tolerance) {
    // Check 1: Determinant should be +1 (proper rotation, not reflection)
    double det = R.determinant();
    if (std::abs(det - 1.0) > tolerance) {
        VLOG(2) << "Invalid rotation matrix: det(R) = " << det
                << " (should be 1.0)";
        return false;
    }
    
    // Check 2: R · R^T should equal I (orthogonality)
    Mat3 product = R * R.transpose();
    Mat3 identity = Mat3::Identity();
    if ((product - identity).norm() > tolerance) {
        VLOG(2) << "Invalid rotation matrix: R·R^T - I norm = "
                << (product - identity).norm() << " (should be ~0)";
        return false;
    }
    
    // Check 3: Each column should have unit norm
    for (int i = 0; i < 3; ++i) {
        double col_norm = R.col(i).norm();
        if (std::abs(col_norm - 1.0) > tolerance) {
            VLOG(2) << "Invalid rotation matrix: column " << i
                    << " norm = " << col_norm << " (should be 1.0)";
            return false;
        }
    }
    
    return true;
}

bool IsValidQuaternion(const Quaternion& q, double tolerance) {
    double norm = q.norm();
    return std::abs(norm - 1.0) < tolerance;
}

// ============================================================================
// ROUND-TRIP VALIDATION
// ============================================================================

double TestRoundTrip_OPK_Matrix_OPK(double omega, double phi, double kappa,
                                    double& error_omega,
                                    double& error_phi,
                                    double& error_kappa) {
    // Convert OPK → Matrix
    Mat3 R = OPK_to_RotationMatrix(omega, phi, kappa, true);
    
    // Convert Matrix → OPK
    double omega2, phi2, kappa2;
    RotationMatrix_to_OPK(R, omega2, phi2, kappa2);
    
    // Compute errors
    error_omega = omega2 - omega;
    error_phi = phi2 - phi;
    error_kappa = kappa2 - kappa;
    
    // Handle angle wrapping for kappa (2π periodicity)
    // Normalize kappa error to [-π, π]
    while (error_kappa > M_PI) error_kappa -= 2 * M_PI;
    while (error_kappa < -M_PI) error_kappa += 2 * M_PI;
    
    // Return max error
    double max_error = std::max({std::abs(error_omega),
                                 std::abs(error_phi),
                                 std::abs(error_kappa)});
    
    return max_error;
}

double TestRoundTrip_OPK_Quaternion_OPK(double omega, double phi, double kappa,
                                        double& error_omega,
                                        double& error_phi,
                                        double& error_kappa) {
    // Convert OPK → Quaternion
    Quaternion q = OPK_to_Quaternion(omega, phi, kappa);
    
    // Convert Quaternion → OPK
    double omega2, phi2, kappa2;
    Quaternion_to_OPK(q, omega2, phi2, kappa2);
    
    // Compute errors
    error_omega = omega2 - omega;
    error_phi = phi2 - phi;
    error_kappa = kappa2 - kappa;
    
    // Handle angle wrapping for kappa
    while (error_kappa > M_PI) error_kappa -= 2 * M_PI;
    while (error_kappa < -M_PI) error_kappa += 2 * M_PI;
    
    // Return max error
    double max_error = std::max({std::abs(error_omega),
                                 std::abs(error_phi),
                                 std::abs(error_kappa)});
    
    return max_error;
}

}  // namespace insight
