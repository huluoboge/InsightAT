#include "cuda/cuda_sfm_state.h"
#include "../camera/camera_utils.h"
#include "../camera/camera_types.h"

#include <cuda_runtime.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace insight;
using namespace insight::cuda;

static void intrinsics_to_float9(const camera::Intrinsics& K, float* out)
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

int main()
{
    const int n_images  = 1;
    const int n_tracks  = 1;
    const int n_obs     = 32;
    const int n_cameras = 1;

    CudaSfMState* gs = gpu_sfm_state_create(n_images, n_tracks, n_obs, n_cameras);
    if (gs == nullptr) {
        std::fprintf(stderr, "gpu_sfm_state_create returned null\n");
        return 1;
    }

    camera::Intrinsics K;
    K.fx = 900.0;
    K.fy = 900.0;
    K.cx = 512.0;
    K.cy = 384.0;
    K.k1 = -0.05;
    K.k2 = 0.01;
    K.k3 = 0.0;
    K.p1 = 0.0001;
    K.p2 = -0.0002;

    std::vector<float> Kf(9);
    intrinsics_to_float9(K, Kf.data());
    gpu_sfm_upload_intrinsics(gs, Kf.data(), 1);

    std::vector<float>   obs_u(n_obs), obs_v(n_obs);
    std::vector<int>     obs_im(n_obs, 0), obs_tr(n_obs, 0), obs_cam(n_obs, 0);
    std::vector<uint8_t> obs_valid(n_obs, 1);
    for (int i = 0; i < n_obs; ++i) {
        obs_u[static_cast<size_t>(i)] = 100.0f + static_cast<float>(i) * 11.7f;
        obs_v[static_cast<size_t>(i)] = 120.0f + static_cast<float>(i) * 9.3f;
    }
    gpu_sfm_upload_observations(gs, obs_u.data(), obs_v.data(), obs_im.data(), obs_tr.data(), obs_cam.data(),
                                obs_valid.data(), n_obs);

    gpu_sfm_state_undistort_all(gs);

    std::vector<float> xn(n_obs), yn(n_obs);
    gpu_sfm_download_obs_norm(gs, xn.data(), yn.data(), n_obs);

    // undistort_point 输出为去畸变像素 (u',v')；GPU d_obs_xn/yn 为归一化平面 (xn,yn)。
    double max_ex = 0.0, max_ey = 0.0, sum_sq = 0.0;
    for (int i = 0; i < n_obs; ++i) {
        double u_px = 0.0, v_px = 0.0;
        camera::undistort_point(K, static_cast<double>(obs_u[static_cast<size_t>(i)]),
                                static_cast<double>(obs_v[static_cast<size_t>(i)]), &u_px, &v_px);
        const double xg = (u_px - K.cx) / K.fx;
        const double yg = (v_px - K.cy) / K.fy;
        const double ex = std::fabs(static_cast<double>(xn[static_cast<size_t>(i)]) - xg);
        const double ey = std::fabs(static_cast<double>(yn[static_cast<size_t>(i)]) - yg);
        max_ex   = std::max(max_ex, ex);
        max_ey   = std::max(max_ey, ey);
        sum_sq += ex * ex + ey * ey;
        if (ex >= 5e-3 || ey >= 5e-3) {
            std::fprintf(stderr, "undistort mismatch i=%d ex=%g ey=%g\n", i, ex, ey);
            gpu_sfm_state_free(gs);
            return 1;
        }
    }

    const double rms = std::sqrt(sum_sq / static_cast<double>(2 * n_obs));
    std::printf("test_cuda_sfm_state — GPU undistort vs CPU golden\n");
    std::printf("  n_obs=%d  tol per coord=5e-3 (normalised plane)\n", n_obs);
    std::printf("  max|xn_gpu-xn_cpu|=%.4g  max|yn_gpu-yn_cpu|=%.4g  RMS combined=%.4g\n", max_ex, max_ey, rms);
    std::printf("OK\n");

    gpu_sfm_state_free(gs);
    return 0;
}
