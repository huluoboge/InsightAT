#include "cuda/cuda_pnp_ransac.cuh"

#include <cuda_runtime.h>
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace insight;
using namespace insight::cuda;

/** Geodesic angle (deg) between rotations: R_rel = R_est * R_true^T */
static double rotation_angle_deg(const Eigen::Matrix3d& R_est, const Eigen::Matrix3d& R_true)
{
    const Eigen::Matrix3d R_rel = R_est * R_true.transpose();
    const double c           = std::min(1.0, std::max(-1.0, (R_rel.trace() - 1.0) * 0.5));
    return std::acos(c) * (180.0 / M_PI);
}

static int run_recovers_pose_no_noise_single_image()
{
    const int n_obs = 80;
    Eigen::Matrix3d R_true;
    R_true = Eigen::AngleAxisd(0.31, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(-0.17, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitX());
    Eigen::Vector3d t_true(0.15, -0.08, 0.35);

    std::vector<float> xn(n_obs), yn(n_obs), xyz(n_obs * 3);
    for (int i = 0; i < n_obs; ++i) {
        Eigen::Vector3d Xw(static_cast<double>(i) * 0.07 - 1.2, std::sin(static_cast<double>(i) * 0.03) * 2.0,
                             3.0 + static_cast<double>(i % 7) * 0.15);
        Eigen::Vector3d Xc = R_true * Xw + t_true;
        const double invz = 1.0 / Xc.z();
        xn[static_cast<size_t>(i)] = static_cast<float>(Xc.x() * invz);
        yn[static_cast<size_t>(i)] = static_cast<float>(Xc.y() * invz);
        xyz[static_cast<size_t>(i) * 3 + 0]     = static_cast<float>(Xw.x());
        xyz[static_cast<size_t>(i) * 3 + 1]     = static_cast<float>(Xw.y());
        xyz[static_cast<size_t>(i) * 3 + 2]     = static_cast<float>(Xw.z());
    }

    float *d_xn = nullptr, *d_yn = nullptr, *d_xyz = nullptr;
    if (cudaMalloc(&d_xn, n_obs * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&d_yn, n_obs * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&d_xyz, n_obs * 3 * sizeof(float)) != cudaSuccess) {
        std::fprintf(stderr, "cudaMalloc failed\n");
        return 1;
    }
    cudaMemcpy(d_xn, xn.data(), n_obs * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_yn, yn.data(), n_obs * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_xyz, xyz.data(), n_obs * 3 * sizeof(float), cudaMemcpyHostToDevice);

    GpuPnpBatchContext* ctx = gpu_pnp_batch_ctx_create(1, 512, n_obs);
    if (ctx == nullptr) {
        std::fprintf(stderr, "gpu_pnp_batch_ctx_create returned null\n");
        cudaFree(d_xn);
        cudaFree(d_yn);
        cudaFree(d_xyz);
        return 1;
    }

    GpuPnpImageDesc desc;
    desc.d_obs_xn = d_xn;
    desc.d_obs_yn = d_yn;
    desc.d_pts3d  = d_xyz;
    desc.n_obs    = n_obs;
    desc.seed     = 4242u;

    const float thr_n_sq = 1e-4f;
    std::vector<float> outR(9), outT(3);
    int out_inl = 0;
    const int n_ok = gpu_pnp_ransac_batch(ctx, &desc, 1, 512, thr_n_sq, outR.data(), outT.data(), &out_inl);
    if (n_ok < 1 || out_inl < n_obs / 2) {
        std::fprintf(stderr, "single-image PnP: n_ok=%d out_inl=%d\n", n_ok, out_inl);
        gpu_pnp_batch_ctx_free(ctx);
        cudaFree(d_xn);
        cudaFree(d_yn);
        cudaFree(d_xyz);
        return 1;
    }

    Eigen::Matrix3d R_est;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R_est(r, c) = static_cast<double>(outR[static_cast<size_t>(r * 3 + c)]);
    Eigen::Vector3d t_est(outT[0], outT[1], outT[2]);

    const double e_R_frob = (R_est - R_true).norm();
    const double e_t      = (t_est - t_true).norm();
    const double ang_deg  = rotation_angle_deg(R_est, R_true);
    if (e_R_frob >= 0.08 || e_t >= 0.15) {
        std::fprintf(stderr, "single-image PnP pose error ||R-R||_F=%g ||t-t||=%g angle=%.3f deg\n", e_R_frob, e_t,
                     ang_deg);
        gpu_pnp_batch_ctx_free(ctx);
        cudaFree(d_xn);
        cudaFree(d_yn);
        cudaFree(d_xyz);
        return 1;
    }

    std::printf("[PnP single] n_obs=%d  RANSAC_iter=512  inliers=%d/%d  thr_n^2=1e-4\n", n_obs, out_inl, n_obs);
    std::printf("            ||R-R_gt||_F=%.4g  ||t-t_gt||=%.4g  geodesic_angle=%.4f deg\n", e_R_frob, e_t,
                ang_deg);

    gpu_pnp_batch_ctx_free(ctx);
    cudaFree(d_xn);
    cudaFree(d_yn);
    cudaFree(d_xyz);
    return 0;
}

static int run_two_images_independent()
{
    const int n0 = 40, n1 = 55;
    // 3D 点需张成体积；共线/近平面会使 P3P 多解，RANSAC 可能选与真值 Frobenius 差很大的等价支。
    auto synth = [](int n, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, std::vector<float>* xn,
                    std::vector<float>* yn, std::vector<float>* xyz) {
        xn->resize(static_cast<size_t>(n));
        yn->resize(static_cast<size_t>(n));
        xyz->resize(static_cast<size_t>(n) * 3u);
        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d Xw(static_cast<double>(i) * 0.07 - 0.8, std::sin(static_cast<double>(i) * 0.11) * 1.4,
                               2.2 + static_cast<double>(i % 9) * 0.12);
            Eigen::Vector3d Xc = R * Xw + t;
            const double invz = 1.0 / Xc.z();
            (*xn)[static_cast<size_t>(i)] = static_cast<float>(Xc.x() * invz);
            (*yn)[static_cast<size_t>(i)] = static_cast<float>(Xc.y() * invz);
            (*xyz)[static_cast<size_t>(i) * 3u + 0] = static_cast<float>(Xw.x());
            (*xyz)[static_cast<size_t>(i) * 3u + 1] = static_cast<float>(Xw.y());
            (*xyz)[static_cast<size_t>(i) * 3u + 2] = static_cast<float>(Xw.z());
        }
    };

    Eigen::Matrix3d R0 = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d t0(0.1, 0.05, 0.2);
    Eigen::Matrix3d R1 = Eigen::AngleAxisd(-0.35, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Vector3d t1(-0.05, 0.12, 0.4);

    std::vector<float> x0, y0, z0, x1, y1, z1;
    synth(n0, R0, t0, &x0, &y0, &z0);
    synth(n1, R1, t1, &x1, &y1, &z1);

    const int M = n0 + n1;
    float *d_pack_x = nullptr, *d_pack_y = nullptr, *d_pack_z = nullptr;
    if (cudaMalloc(&d_pack_x, M * sizeof(float)) != cudaSuccess || cudaMalloc(&d_pack_y, M * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&d_pack_z, M * 3 * sizeof(float)) != cudaSuccess) {
        std::fprintf(stderr, "cudaMalloc (pack) failed\n");
        return 1;
    }
    cudaMemcpy(d_pack_x, x0.data(), n0 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_pack_y, y0.data(), n0 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_pack_z, z0.data(), n0 * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_pack_x + n0, x1.data(), n1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_pack_y + n0, y1.data(), n1 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_pack_z + n0 * 3, z1.data(), n1 * 3 * sizeof(float), cudaMemcpyHostToDevice);

    GpuPnpBatchContext* ctx = gpu_pnp_batch_ctx_create(2, 400, M);
    if (ctx == nullptr) {
        std::fprintf(stderr, "gpu_pnp_batch_ctx_create (2 img) returned null\n");
        cudaFree(d_pack_x);
        cudaFree(d_pack_y);
        cudaFree(d_pack_z);
        return 1;
    }
    GpuPnpImageDesc descs[2];
    descs[0] = {d_pack_x, d_pack_y, d_pack_z, n0, 11u};
    descs[1] = {d_pack_x + n0, d_pack_y + n0, d_pack_z + n0 * 3, n1, 99u};

    std::vector<float> outR(18), outT(6);
    std::vector<int> inl(2);
    const int n_ok = gpu_pnp_ransac_batch(ctx, descs, 2, 400, 2e-4f, outR.data(), outT.data(), inl.data());
    if (n_ok != 2 || inl[0] < n0 / 3 || inl[1] < n1 / 3) {
        std::fprintf(stderr, "two-image PnP: n_ok=%d inl0=%d inl1=%d\n", n_ok, inl[0], inl[1]);
        gpu_pnp_batch_ctx_free(ctx);
        cudaFree(d_pack_x);
        cudaFree(d_pack_y);
        cudaFree(d_pack_z);
        return 1;
    }

    Eigen::Matrix3d R0e, R1e;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            R0e(r, c) = static_cast<double>(outR[static_cast<size_t>(r * 3 + c)]);
            R1e(r, c) = static_cast<double>(outR[9 + static_cast<size_t>(r * 3 + c)]);
        }
    Eigen::Vector3d t0e(outT[0], outT[1], outT[2]);
    Eigen::Vector3d t1e(outT[3], outT[4], outT[5]);
    const double eR0 = (R0e - R0).norm();
    const double eR1 = (R1e - R1).norm();
    const double et0 = (t0e - t0).norm();
    const double et1 = (t1e - t1).norm();
    const double a0  = rotation_angle_deg(R0e, R0);
    const double a1  = rotation_angle_deg(R1e, R1);

    if (eR0 >= 0.12 || eR1 >= 0.12) {
        std::fprintf(stderr, "two-image PnP: ||R-R||_F img0=%.4g img1=%.4g  angle_deg %.4f %.4f\n", eR0, eR1, a0, a1);
        gpu_pnp_batch_ctx_free(ctx);
        cudaFree(d_pack_x);
        cudaFree(d_pack_y);
        cudaFree(d_pack_z);
        return 1;
    }

    std::printf("[PnP batch K=2] RANSAC_iter=400  thr_n^2=2e-4\n");
    std::printf("  image0: n=%d inliers=%d  ||R-R_gt||_F=%.4g  angle=%.4f deg  ||t-t_gt||=%.4g\n", n0, inl[0], eR0,
                a0, et0);
    std::printf("  image1: n=%d inliers=%d  ||R-R_gt||_F=%.4g  angle=%.4f deg  ||t-t_gt||=%.4g\n", n1, inl[1], eR1,
                a1, et1);

    gpu_pnp_batch_ctx_free(ctx);
    cudaFree(d_pack_x);
    cudaFree(d_pack_y);
    cudaFree(d_pack_z);
    return 0;
}

int main()
{
    std::printf("test_cuda_pnp_ransac — CUDA batch P3P RANSAC self-check\n");
    if (int e = run_recovers_pose_no_noise_single_image())
        return e;
    if (int e = run_two_images_independent())
        return e;
    std::printf("OK — all PnP checks passed\n");
    return 0;
}
