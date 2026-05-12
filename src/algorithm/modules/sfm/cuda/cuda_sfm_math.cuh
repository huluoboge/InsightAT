/**
 * @file  cuda_sfm_math.cuh
 * @brief Shared host-side flat-float conversion helpers for CudaSfMState uploads.
 *
 * These are pure host functions (no CUDA device code); include in any .cu or .cpp
 * that needs to flatten Eigen / Intrinsics types into the float arrays expected by
 * gpu_sfm_upload_* and the triangulation / resection batch APIs.
 */

#pragma once
#ifndef INSIGHT_CUDA_SFM_MATH_CUH
#define INSIGHT_CUDA_SFM_MATH_CUH

#include "../../camera/camera_types.h"

#include <Eigen/Core>

namespace insight {
namespace cuda {

// ─────────────────────────────────────────────────────────────────────────────
// Row-major 3×3 rotation matrix → float[9]
// ─────────────────────────────────────────────────────────────────────────────

inline void eigen_R_to_float9(const Eigen::Matrix3d& R, float* out)
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r * 3 + c] = static_cast<float>(R(r, c));
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera centre vector → float[3]
// ─────────────────────────────────────────────────────────────────────────────

inline void eigen_C_to_float3(const Eigen::Vector3d& C, float* out)
{
    out[0] = static_cast<float>(C(0));
    out[1] = static_cast<float>(C(1));
    out[2] = static_cast<float>(C(2));
}

// ─────────────────────────────────────────────────────────────────────────────
// Brown-Conrady intrinsics → float[9]  {fx,fy,cx,cy,k1,k2,k3,p1,p2}
// ─────────────────────────────────────────────────────────────────────────────

inline void intrinsics_to_float9(const camera::Intrinsics& K, float* out)
{
    out[0] = static_cast<float>(K.fx);
    out[1] = static_cast<float>(K.fy);
    out[2] = static_cast<float>(K.cx);
    out[3] = static_cast<float>(K.cy);
    out[4] = static_cast<float>(K.k1);
    out[5] = static_cast<float>(K.k2);
    out[6] = static_cast<float>(K.k3);
    out[7] = static_cast<float>(K.p1);
    out[8] = static_cast<float>(K.p2);
}

} // namespace cuda
} // namespace insight

#endif // INSIGHT_CUDA_SFM_MATH_CUH
