/**
 * @file cuda_pnp_lambdatwist_device.cuh
 * @brief __device__ port of PoseLib `p3p_lambdatwist` (Persson & Nordberg, ECCV 2018).
 *
 * Copyright (c) 2020, Viktor Larsson — same BSD terms as third_party/PoseLib.
 * Ported to CUDA __device__ code (double arithmetic) for batch PnP RANSAC.
 */

#pragma once

#include <cuda_runtime.h>
#include <math.h>

namespace insight {
namespace cuda {
namespace p3p_dev {

struct Vec3 {
    double x, y, z;
    __device__ __forceinline__ Vec3() : x(0), y(0), z(0) {}
    __device__ __forceinline__ Vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
    __device__ __forceinline__ Vec3 operator-(const Vec3& o) const {
        return Vec3(x - o.x, y - o.y, z - o.z);
    }
    __device__ __forceinline__ Vec3 operator+(const Vec3& o) const {
        return Vec3(x + o.x, y + o.y, z + o.z);
    }
    __device__ __forceinline__ Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    __device__ __forceinline__ double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    __device__ __forceinline__ Vec3 cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }
    __device__ __forceinline__ double squaredNorm() const { return x * x + y * y + z * z; }
};

struct Mat3 {
    double m[9]; // row-major

    __device__ __forceinline__ static Mat3 Zero() {
        Mat3 A;
        for (int i = 0; i < 9; ++i) A.m[i] = 0;
        return A;
    }
    __device__ __forceinline__ double& at(int r, int c) { return m[r * 3 + c]; }
    __device__ __forceinline__ double at(int r, int c) const { return m[r * 3 + c]; }
    __device__ __forceinline__ Vec3 col(int c) const { return Vec3(at(0, c), at(1, c), at(2, c)); }
    __device__ __forceinline__ void setCol(int c, const Vec3& v) {
        at(0, c) = v.x;
        at(1, c) = v.y;
        at(2, c) = v.z;
    }
};

__device__ __forceinline__ Mat3 operator+(const Mat3& A, const Mat3& B) {
    Mat3 C;
    for (int i = 0; i < 9; ++i) C.m[i] = A.m[i] + B.m[i];
    return C;
}

__device__ __forceinline__ Mat3 operator*(const Mat3& A, double g) {
    Mat3 C;
    for (int i = 0; i < 9; ++i) C.m[i] = A.m[i] * g;
    return C;
}

__device__ __forceinline__ Mat3 matMul(const Mat3& A, const Mat3& B) {
    Mat3 C = Mat3::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C.at(i, j) += A.at(i, k) * B.at(k, j);
    return C;
}

__device__ __forceinline__ Vec3 matVec(const Mat3& A, const Vec3& v) {
    return Vec3(A.at(0, 0) * v.x + A.at(0, 1) * v.y + A.at(0, 2) * v.z,
                A.at(1, 0) * v.x + A.at(1, 1) * v.y + A.at(1, 2) * v.z,
                A.at(2, 0) * v.x + A.at(2, 1) * v.y + A.at(2, 2) * v.z);
}

__device__ __forceinline__ void compute_eig3x3known0(const Mat3& M, Mat3& E, double& sig1, double& sig2)
{
    double p1 = -M.at(0, 0) - M.at(1, 1) - M.at(2, 2);
    double p0 = -M.at(0, 1) * M.at(0, 1) - M.at(0, 2) * M.at(0, 2) - M.at(1, 2) * M.at(1, 2) +
                M.at(0, 0) * (M.at(1, 1) + M.at(2, 2)) + M.at(1, 1) * M.at(2, 2);

    double disc = sqrt(p1 * p1 / 4.0 - p0);
    double tmp  = -p1 / 2.0;
    sig1        = tmp + disc;
    sig2        = tmp - disc;

    if (fabs(sig1) < fabs(sig2)) {
        double t = sig1;
        sig1     = sig2;
        sig2     = t;
    }

    double c = sig1 * sig1 + M.at(0, 0) * M.at(1, 1) - sig1 * (M.at(0, 0) + M.at(1, 1)) - M.at(0, 1) * M.at(0, 1);
    double a1 = (sig1 * M.at(0, 2) + M.at(0, 1) * M.at(1, 2) - M.at(0, 2) * M.at(1, 1)) / c;
    double a2 = (sig1 * M.at(1, 2) + M.at(0, 1) * M.at(0, 2) - M.at(0, 0) * M.at(1, 2)) / c;
    double n  = 1.0 / sqrt(1.0 + a1 * a1 + a2 * a2);
    E.setCol(0, Vec3(a1 * n, a2 * n, n));

    c  = sig2 * sig2 + M.at(0, 0) * M.at(1, 1) - sig2 * (M.at(0, 0) + M.at(1, 1)) - M.at(0, 1) * M.at(0, 1);
    a1 = (sig2 * M.at(0, 2) + M.at(0, 1) * M.at(1, 2) - M.at(0, 2) * M.at(1, 1)) / c;
    a2 = (sig2 * M.at(1, 2) + M.at(0, 1) * M.at(0, 2) - M.at(0, 0) * M.at(1, 2)) / c;
    n  = 1.0 / sqrt(1.0 + a1 * a1 + a2 * a2);
    E.setCol(1, Vec3(a1 * n, a2 * n, n));
}

__device__ __forceinline__ void dev_refine_lambda(double& lambda1, double& lambda2, double& lambda3,
                                                     double a12, double a13, double a23, double b12, double b13,
                                                     double b23)
{
    for (int iter = 0; iter < 5; ++iter) {
        double r1 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda2 * b12 + lambda2 * lambda2 - a12);
        double r2 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda3 * b13 + lambda3 * lambda3 - a13);
        double r3 = (lambda2 * lambda2 - 2.0 * lambda2 * lambda3 * b23 + lambda3 * lambda3 - a23);
        if (fabs(r1) + fabs(r2) + fabs(r3) < 1e-10)
            return;
        double x11 = lambda1 - lambda2 * b12;
        double x12 = lambda2 - lambda1 * b12;
        double x21 = lambda1 - lambda3 * b13;
        double x23 = lambda3 - lambda1 * b13;
        double x32 = lambda2 - lambda3 * b23;
        double x33 = lambda3 - lambda2 * b23;
        double detJ = 0.5 / (x11 * x23 * x32 + x12 * x21 * x33);
        lambda1 += (-x23 * x32 * r1 - x12 * x33 * r2 + x12 * x23 * r3) * detJ;
        lambda2 += (-x21 * x33 * r1 + x11 * x33 * r2 - x11 * x23 * r3) * detJ;
        lambda3 += (x21 * x32 * r1 - x11 * x32 * r2 - x12 * x21 * r3) * detJ;
    }
}

/// Returns number of solutions (0–4). R_out[k*9..], t_out[k*3..] row-major R.
__device__ int dev_p3p_lambdatwist(const Vec3& x0, const Vec3& x1, const Vec3& x2, const Vec3& X0, const Vec3& X1,
                                   const Vec3& X2, float* R_out, float* t_out, int max_sols)
{
    Vec3 dX12 = X0 - X1;
    Vec3 dX13 = X0 - X2;
    Vec3 dX23 = X1 - X2;

    double a12 = dX12.squaredNorm();
    double b12 = x0.dot(x1);

    double a13 = dX13.squaredNorm();
    double b13 = x0.dot(x2);

    double a23 = dX23.squaredNorm();
    double b23 = x1.dot(x2);

    double a23b12 = a23 * b12;
    double a12b23 = a12 * b23;
    double a23b13 = a23 * b13;
    double a13b23 = a13 * b23;

    // Eigen::Matrix3d << uses column-major fill order (same layout as PoseLib source).
    Mat3 D1 = Mat3::Zero();
    D1.at(0, 0) = a23;
    D1.at(1, 0) = -a23b12;
    D1.at(2, 0) = 0.0;
    D1.at(0, 1) = -a23b12;
    D1.at(1, 1) = a23 - a12;
    D1.at(2, 1) = a12b23;
    D1.at(0, 2) = 0.0;
    D1.at(1, 2) = a12b23;
    D1.at(2, 2) = -a12;

    Mat3 D2 = Mat3::Zero();
    D2.at(0, 0) = a23;
    D2.at(1, 0) = 0.0;
    D2.at(2, 0) = -a23b13;
    D2.at(0, 1) = 0.0;
    D2.at(1, 1) = -a13;
    D2.at(2, 1) = a13b23;
    D2.at(0, 2) = -a23b13;
    D2.at(1, 2) = a13b23;
    D2.at(2, 2) = a23 - a13;

    Mat3 DX1;
    DX1.setCol(0, D1.col(1).cross(D1.col(2)));
    DX1.setCol(1, D1.col(2).cross(D1.col(0)));
    DX1.setCol(2, D1.col(0).cross(D1.col(1)));

    Mat3 DX2;
    DX2.setCol(0, D2.col(1).cross(D2.col(2)));
    DX2.setCol(1, D2.col(2).cross(D2.col(0)));
    DX2.setCol(2, D2.col(0).cross(D2.col(1)));

    double c3 = D2.col(0).dot(DX2.col(0));
    double c2 = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            c2 += D1.at(i, j) * DX2.at(i, j);
    double c1 = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            c1 += D2.at(i, j) * DX1.at(i, j);
    double c0 = D1.col(0).dot(DX1.col(0));

    const double c3inv = 1.0 / c3;
    c2 *= c3inv;
    c1 *= c3inv;
    c0 *= c3inv;

    double a_c = c1 - c2 * c2 / 3.0;
    double b_c = (2.0 * c2 * c2 * c2 - 9.0 * c2 * c1) / 27.0 + c0;
    double c_disc = b_c * b_c / 4.0 + a_c * a_c * a_c / 27.0;
    double gamma;
    if (c_disc > 0) {
        c_disc = sqrt(c_disc);
        b_c *= -0.5;
        gamma = cbrt(b_c + c_disc) + cbrt(b_c - c_disc) - c2 / 3.0;
    } else {
        c_disc = 3.0 * b_c / (2.0 * a_c) * sqrt(-3.0 / a_c);
        gamma = 2.0 * sqrt(-a_c / 3.0) * cos(acos(c_disc) / 3.0) - c2 / 3.0;
    }

    double f = gamma * gamma * gamma + c2 * gamma * gamma + c1 * gamma + c0;
    double df = 3.0 * gamma * gamma + 2.0 * c2 * gamma + c1;
    gamma = gamma - f / df;

    Mat3 D0 = Mat3::Zero();
    for (int i = 0; i < 9; ++i)
        D0.m[i] = D1.m[i] + D2.m[i] * gamma;

    Mat3 E;
    double sig1, sig2;
    compute_eig3x3known0(D0, E, sig1, sig2);

    double s = sqrt(-sig2 / sig1);
    double lambda1, lambda2, lambda3;

    Mat3 XX;
    XX.setCol(0, dX12);
    XX.setCol(1, dX13);
    XX.setCol(2, dX12.cross(dX13));
    // inverse 3x3
    double det = XX.at(0, 0) * (XX.at(1, 1) * XX.at(2, 2) - XX.at(1, 2) * XX.at(2, 1)) -
                 XX.at(0, 1) * (XX.at(1, 0) * XX.at(2, 2) - XX.at(1, 2) * XX.at(2, 0)) +
                 XX.at(0, 2) * (XX.at(1, 0) * XX.at(2, 1) - XX.at(1, 1) * XX.at(2, 0));
    if (fabs(det) < 1e-14)
        return 0;
    double inv_det = 1.0 / det;
    Mat3 XXi;
    XXi.at(0, 0) = (XX.at(1, 1) * XX.at(2, 2) - XX.at(1, 2) * XX.at(2, 1)) * inv_det;
    XXi.at(0, 1) = (XX.at(0, 2) * XX.at(2, 1) - XX.at(0, 1) * XX.at(2, 2)) * inv_det;
    XXi.at(0, 2) = (XX.at(0, 1) * XX.at(1, 2) - XX.at(0, 2) * XX.at(1, 1)) * inv_det;
    XXi.at(1, 0) = (XX.at(1, 2) * XX.at(2, 0) - XX.at(1, 0) * XX.at(2, 2)) * inv_det;
    XXi.at(1, 1) = (XX.at(0, 0) * XX.at(2, 2) - XX.at(0, 2) * XX.at(2, 0)) * inv_det;
    XXi.at(1, 2) = (XX.at(1, 0) * XX.at(0, 2) - XX.at(0, 0) * XX.at(1, 2)) * inv_det;
    XXi.at(2, 0) = (XX.at(1, 0) * XX.at(2, 1) - XX.at(1, 1) * XX.at(2, 0)) * inv_det;
    XXi.at(2, 1) = (XX.at(0, 1) * XX.at(2, 0) - XX.at(0, 0) * XX.at(2, 1)) * inv_det;
    XXi.at(2, 2) = (XX.at(0, 0) * XX.at(1, 1) - XX.at(0, 1) * XX.at(1, 0)) * inv_det;

    const double TOL_DOUBLE_ROOT = 1e-12;
    int n_out = 0;

    for (int s_flip = 0; s_flip < 2; ++s_flip) {
        const double s_use = (s_flip == 0) ? s : -s;
        double u1 = E.at(0, 0) - s_use * E.at(0, 1);
        double u2 = E.at(1, 0) - s_use * E.at(1, 1);
        double u3 = E.at(2, 0) - s_use * E.at(2, 1);

        bool switch_12 = fabs(u1) < fabs(u2);

        if (switch_12) {
            double w0 = -u1 / u2;
            double w1 = -u3 / u2;
            double ca = -a13 * w1 * w1 + 2 * a13b23 * w1 - a13 + a23;
            double cb = 2 * a13b23 * w0 - 2 * a23b13 - 2 * a13 * w0 * w1;
            double cc = -a13 * w0 * w0 + a23;

            double b2m4ac = cb * cb - 4.0 * ca * cc;
            if (b2m4ac < -TOL_DOUBLE_ROOT)
                continue;
            double sq = sqrt(fmax(0.0, b2m4ac));

            double tau = (cb > 0) ? (2.0 * cc) / (-cb - sq) : (2.0 * cc) / (-cb + sq);

            for (int tau_flip = 0; tau_flip < 2; ++tau_flip, tau = cc / (ca * tau)) {
                if (tau > 0) {
                    lambda1 = sqrt(a13 / (tau * (tau - 2.0 * b13) + 1.0));
                    lambda3 = tau * lambda1;
                    lambda2 = w0 * lambda1 + w1 * lambda3;
                    if (lambda2 < 0)
                        continue;

                    dev_refine_lambda(lambda1, lambda2, lambda3, a12, a13, a23, b12, b13, b23);
                    Vec3 v1 = x0 * lambda1 - x1 * lambda2;
                    Vec3 v2 = x0 * lambda1 - x2 * lambda3;
                    Mat3 YY;
                    YY.setCol(0, v1);
                    YY.setCol(1, v2);
                    YY.setCol(2, v1.cross(v2));
                    Mat3 Rm = matMul(YY, XXi);
                    Vec3 tv = x0 * lambda1 - matVec(Rm, X0);
                    if (n_out < max_sols) {
                        for (int r = 0; r < 3; ++r)
                            for (int c = 0; c < 3; ++c)
                                R_out[n_out * 9 + r * 3 + c] = static_cast<float>(Rm.at(r, c));
                        t_out[n_out * 3 + 0]     = static_cast<float>(tv.x);
                        t_out[n_out * 3 + 1]     = static_cast<float>(tv.y);
                        t_out[n_out * 3 + 2]     = static_cast<float>(tv.z);
                        ++n_out;
                    }
                }

                if (b2m4ac < TOL_DOUBLE_ROOT)
                    break;
            }

        } else {
            double w0 = -u2 / u1;
            double w1 = -u3 / u1;
            double ca = (a13 - a12) * w1 * w1 + 2.0 * a12 * b13 * w1 - a12;
            double cb = -2.0 * a13 * b12 * w1 + 2.0 * a12 * b13 * w0 - 2.0 * w0 * w1 * (a12 - a13);
            double cc = (a13 - a12) * w0 * w0 - 2.0 * a13 * b12 * w0 + a13;
            double b2m4ac = cb * cb - 4.0 * ca * cc;
            if (b2m4ac < -TOL_DOUBLE_ROOT)
                continue;
            double sq = sqrt(fmax(0.0, b2m4ac));
            double tau = (cb > 0) ? (2.0 * cc) / (-cb - sq) : (2.0 * cc) / (-cb + sq);
            for (int tau_flip = 0; tau_flip < 2; ++tau_flip, tau = cc / (ca * tau)) {
                if (tau > 0) {
                    lambda2 = sqrt(a23 / (tau * (tau - 2.0 * b23) + 1.0));
                    lambda3 = tau * lambda2;
                    lambda1 = w0 * lambda2 + w1 * lambda3;

                    if (lambda1 < 0)
                        continue;
                    dev_refine_lambda(lambda1, lambda2, lambda3, a12, a13, a23, b12, b13, b23);
                    Vec3 v1 = x0 * lambda1 - x1 * lambda2;
                    Vec3 v2 = x0 * lambda1 - x2 * lambda3;
                    Mat3 YY;
                    YY.setCol(0, v1);
                    YY.setCol(1, v2);
                    YY.setCol(2, v1.cross(v2));
                    Mat3 Rm = matMul(YY, XXi);
                    Vec3 tv = x0 * lambda1 - matVec(Rm, X0);
                    if (n_out < max_sols) {
                        for (int r = 0; r < 3; ++r)
                            for (int c = 0; c < 3; ++c)
                                R_out[n_out * 9 + r * 3 + c] = static_cast<float>(Rm.at(r, c));
                        t_out[n_out * 3 + 0]     = static_cast<float>(tv.x);
                        t_out[n_out * 3 + 1]     = static_cast<float>(tv.y);
                        t_out[n_out * 3 + 2]     = static_cast<float>(tv.z);
                        ++n_out;
                    }
                }
                if (b2m4ac < TOL_DOUBLE_ROOT)
                    break;
            }
        }

        if (n_out > 0)
            break;
    }

    return n_out;
}

} // namespace p3p_dev
} // namespace cuda
} // namespace insight
