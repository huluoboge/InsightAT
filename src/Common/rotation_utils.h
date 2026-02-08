/**
 * @file rotation_utils.h
 * @brief Rotation utilities for photogrammetric pose representation
 *
 * STANDARD: ISPRS Photogrammetry + IEEE 1571-2006 (Navigation)
 * See doc/rotation/ROTATION_STANDARDS.md for complete specification
 *
 * This module provides:
 * 1. OmegaPhiKappa (OPK) ↔ Rotation Matrix conversion
 * 2. OmegaPhiKappa (OPK) ↔ Quaternion conversion
 * 3. OmegaPhiKappa (OPK) ↔ YawPitchRoll (YPR) conversion
 * 4. Gimbal lock detection and handling
 *
 * KEY CONVENTION:
 * - OmegaPhiKappa uses EXTRINSIC rotation (fixed world axes)
 *   R = R_z(κ) · R_y(φ) · R_x(ω)  [Z-Y-X order, extrinsic]
 * - Transform: p_camera = R · p_world  [World → Camera, passive rotation]
 * - YawPitchRoll uses INTRINSIC rotation (body-fixed axes)
 *   R = R_z(ψ) · R_y(θ) · R_x(φ)  [Z-Y-X order, intrinsic]
 * - Transform: p_nav = R · p_body  [Body → Nav, passive rotation]
 *
 * COORDINATE SYSTEMS:
 * - Photogrammetry: Camera frame = {X-right, Y-down, Z-forward}
 * - Navigation: Body frame = {X-forward, Y-right, Z-down} (typically)
 */

#ifndef INSIGHT_COMMON_ROTATION_UTILS_H
#define INSIGHT_COMMON_ROTATION_UTILS_H

#include "common_global.h"
#include "numeric.h"

#include <cmath>
#include <stdexcept>

namespace insight {

// Forward declare validation functions (no default parameters here)
bool IsValidRotationMatrix(const Mat3& R, double tolerance);
bool IsValidQuaternion(const Quaternion& q, double tolerance);

// ============================================================================
// ENUMERATIONS - Standard from CODE_ANALYSIS_ROTATION.md
// ============================================================================

/**
 * World coordinate system type.
 * Indicates what type of coordinates are stored in DBPose::x, y, z
 */
enum class WorldCoordinateSystem {
    ECEF = 0,              ///< Earth-Centered Earth-Fixed (global)
    ProjectedUTM = 1,      ///< UTM or other projected coordinate system
    Local_ENU = 2,         ///< East-North-Up local frame
    Local_NED = 3          ///< North-East-Down local frame
};

/**
 * Rotation matrix direction.
 * Specifies what transformation the rotation matrix represents
 */
enum class RotationMatrixType {
    World_to_Camera = 0,   ///< Standard: p_camera = R · p_world (passive)
    Camera_to_World = 1    ///< Inverse: p_world = R · p_camera (passive)
};

/**
 * Euler angle convention.
 * Indicates which variant of OmegaPhiKappa is used
 */
enum class EulerAngleConvention {
    OmegaPhiKappa = 0,     ///< Standard photogrammetric: Z-Y-X extrinsic
    PhiOmegaKappa = 1      ///< Variant: different axis order
};

// ============================================================================
// GIMBAL LOCK DETECTION
// ============================================================================

/**
 * Check if a phi angle is near gimbal lock condition.
 *
 * For OmegaPhiKappa (Z-Y-X extrinsic), gimbal lock occurs at φ = ±π/2
 * This function returns true if phi is within a threshold of the singularity.
 *
 * @param phi_rad  Phi angle in radians
 * @param threshold Distance from ±π/2 to consider as gimbal lock (default: 0.1 rad ≈ 5.7°)
 * @return true if gimbal lock is detected, false otherwise
 */
bool IsGimbalLockRisk(double phi_rad, double threshold = 0.1);

/**
 * Get the magnitude of gimbal lock singularity for a given phi.
 *
 * Returns 0.0 when safe, increases as phi approaches ±π/2.
 * Useful for monitoring solution stability.
 *
 * @param phi_rad  Phi angle in radians
 * @return Singularity magnitude (0.0 = safe, >1.0 = singularity)
 */
double GetGimbalLockMagnitude(double phi_rad);

// ============================================================================
// CONVERSION: OmegaPhiKappa ↔ Rotation Matrix
// ============================================================================

/**
 * Convert OmegaPhiKappa Euler angles to rotation matrix.
 *
 * STANDARD: ISPRS Photogrammetry
 * Convention: Z-Y-X extrinsic rotation (fixed world axes)
 * Formula: R = R_z(κ) · R_y(φ) · R_x(ω)
 *
 * Example:
 * ```cpp
 * double omega = 0.05, phi = 0.1, kappa = 0.15;  // in radians
 * Mat3 R = OPK_to_RotationMatrix(omega, phi, kappa);
 * // R transforms world coordinates to camera coordinates
 * // p_camera = R * p_world
 * ```
 *
 * @param omega  Roll angle (rotation around X) in radians
 * @param phi    Pitch angle (rotation around Y) in radians
 * @param kappa  Yaw angle (rotation around Z) in radians
 * @return 3×3 rotation matrix
 * @throw std::invalid_argument if gimbal lock is detected and allow_gimbal_lock=false
 */
Mat3 OPK_to_RotationMatrix(double omega, double phi, double kappa,
                           bool allow_gimbal_lock = false);

/**
 * Convert rotation matrix to OmegaPhiKappa Euler angles.
 *
 * Extracts ω, φ, κ from a 3×3 rotation matrix.
 * Note: Multiple solutions exist when gimbal lock is near. This implementation
 * returns the principal solution.
 *
 * Example:
 * ```cpp
 * Mat3 R = OPK_to_RotationMatrix(0.05, 0.1, 0.15);
 * double omega, phi, kappa;
 * RotationMatrix_to_OPK(R, omega, phi, kappa);
 * ```
 *
 * @param R      3×3 rotation matrix
 * @param[out] omega  Roll angle in radians
 * @param[out] phi    Pitch angle in radians
 * @param[out] kappa  Yaw angle in radians
 * @throw std::invalid_argument if matrix is not a valid rotation matrix
 */
void RotationMatrix_to_OPK(const Mat3& R,
                           double& omega,
                           double& phi,
                           double& kappa);

// ============================================================================
// CONVERSION: OmegaPhiKappa ↔ Quaternion
// ============================================================================

/**
 * Convert OmegaPhiKappa Euler angles to unit quaternion.
 *
 * Quaternions avoid gimbal lock and are preferred for interpolation and
 * numerical optimization. This represents the same rotation as the matrix form.
 *
 * Quaternion convention: q = [x, y, z, w] where w is the scalar part
 * Implementation uses q = [qx, qy, qz, qw]
 *
 * @param omega  Roll angle in radians
 * @param phi    Pitch angle in radians
 * @param kappa  Yaw angle in radians
 * @return Unit quaternion (Eigen::Quaternion<double>)
 *
 * Example:
 * ```cpp
 * Quaternion q = OPK_to_Quaternion(0.05, 0.1, 0.15);
 * // q represents the same rotation as OPK_to_RotationMatrix(0.05, 0.1, 0.15)
 * ```
 */
Quaternion OPK_to_Quaternion(double omega, double phi, double kappa);

/**
 * Convert unit quaternion to OmegaPhiKappa Euler angles.
 *
 * Inverse of OPK_to_Quaternion().
 *
 * @param q       Unit quaternion
 * @param[out] omega  Roll angle in radians
 * @param[out] phi    Pitch angle in radians
 * @param[out] kappa  Yaw angle in radians
 * @throw std::invalid_argument if quaternion is not normalized
 */
void Quaternion_to_OPK(const Quaternion& q,
                       double& omega,
                       double& phi,
                       double& kappa);

// ============================================================================
// CONVERSION: OmegaPhiKappa ↔ YawPitchRoll
// ============================================================================

/**
 * Convert OmegaPhiKappa (photogrammetry) to YawPitchRoll (navigation).
 *
 * STANDARDS:
 * - OPK: ISPRS standard, Z-Y-X extrinsic (fixed world axes)
 * - YPR: AIAA R-004-1992, IEEE 1571-2006, Z-Y-X intrinsic (body-fixed axes)
 *
 * KEY DIFFERENCES:
 * - OPK uses EXTRINSIC rotation: R_opk = R_z(κ) · R_y(φ) · R_x(ω)  [fixed axes]
 * - YPR uses INTRINSIC rotation: R_ypr = R_z(ψ) · R_y(θ) · R_x(φ)  [body-fixed]
 * - For the SAME rotation matrix, the angle values differ!
 *
 * COORDINATE FRAMES:
 * - Photogrammetry camera: {X-right, Y-down, Z-forward}
 * - Navigation body: {X-forward, Y-right, Z-down} (typically)
 *
 * This function handles the frame transformation AND the angle convention difference.
 *
 * @param opk_omega  OPK roll angle (rad)
 * @param opk_phi    OPK pitch angle (rad)
 * @param opk_kappa  OPK yaw angle (rad)
 * @param[out] ypr_yaw    YPR yaw angle (rad)
 * @param[out] ypr_pitch  YPR pitch angle (rad)
 * @param[out] ypr_roll   YPR roll angle (rad)
 *
 * Example:
 * ```cpp
 * double yaw, pitch, roll;
 * OPK_to_YPR(0.05, 0.1, 0.15, yaw, pitch, roll);
 * // Now yaw, pitch, roll are in navigation convention
 * ```
 */
void OPK_to_YPR(double opk_omega, double opk_phi, double opk_kappa,
                double& ypr_yaw, double& ypr_pitch, double& ypr_roll);

/**
 * Convert YawPitchRoll (navigation) to OmegaPhiKappa (photogrammetry).
 *
 * Inverse of OPK_to_YPR().
 *
 * @param ypr_yaw    YPR yaw angle (rad)
 * @param ypr_pitch  YPR pitch angle (rad)
 * @param ypr_roll   YPR roll angle (rad)
 * @param[out] opk_omega  OPK roll angle (rad)
 * @param[out] opk_phi    OPK pitch angle (rad)
 * @param[out] opk_kappa  OPK yaw angle (rad)
 */
void YPR_to_OPK(double ypr_yaw, double ypr_pitch, double ypr_roll,
                double& opk_omega, double& opk_phi, double& opk_kappa);

// ============================================================================
// VALIDATION
// ============================================================================

/**
 * Validate that a matrix is a proper rotation matrix.
 *
 * Checks:
 * 1. det(R) = +1 (proper rotation, not reflection)
 * 2. R · R^T = I (orthogonality)
 * 3. ||R·u_i|| = 1 for all column vectors u_i
 *
 * @param R       3×3 matrix to validate
 * @param tolerance  Allowed deviation from ideal values (default: 1e-6)
 * @return true if matrix is a valid rotation matrix
 *
 * Example:
 * ```cpp
 * Mat3 R = OPK_to_RotationMatrix(0.05, 0.1, 0.15);
 * if (!IsValidRotationMatrix(R)) {
 *     LOG(ERROR) << "Invalid rotation matrix!";
 * }
 * ```
 */
bool IsValidRotationMatrix(const Mat3& R, double tolerance = 1e-6);

/**
 * Validate that a quaternion is properly normalized.
 *
 * @param q        Quaternion to validate
 * @param tolerance  Allowed deviation from unity norm (default: 1e-6)
 * @return true if quaternion is normalized
 */
bool IsValidQuaternion(const Quaternion& q, double tolerance = 1e-6);

// ============================================================================
// ROUND-TRIP VALIDATION
// ============================================================================

/**
 * Test round-trip conversion: OPK → Matrix → OPK
 *
 * Useful for validation and testing. Computes the error in angles after
 * converting OPK to rotation matrix and back.
 *
 * @param omega  Original roll angle (rad)
 * @param phi    Original pitch angle (rad)
 * @param kappa  Original yaw angle (rad)
 * @param[out] error_omega  Error in omega (rad)
 * @param[out] error_phi    Error in phi (rad)
 * @param[out] error_kappa  Error in kappa (rad)
 * @return Max error among all three angles (rad)
 *
 * Example:
 * ```cpp
 * double err_omega, err_phi, err_kappa;
 * double max_err = TestRoundTrip_OPK_Matrix_OPK(0.05, 0.1, 0.15,
 *                                               err_omega, err_phi, err_kappa);
 * if (max_err > 1e-8) {
 *     LOG(WARNING) << "Round-trip error too large: " << max_err;
 * }
 * ```
 */
double TestRoundTrip_OPK_Matrix_OPK(double omega, double phi, double kappa,
                                    double& error_omega,
                                    double& error_phi,
                                    double& error_kappa);

/**
 * Test round-trip conversion: OPK → Quaternion → OPK
 */
double TestRoundTrip_OPK_Quaternion_OPK(double omega, double phi, double kappa,
                                       double& error_omega,
                                       double& error_phi,
                                       double& error_kappa);

}  // namespace insight

#endif  // INSIGHT_COMMON_ROTATION_UTILS_H
