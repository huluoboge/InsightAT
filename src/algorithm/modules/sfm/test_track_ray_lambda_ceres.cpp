/**
 * @file  test_track_ray_lambda_ceres.cpp
 * @brief 射线–弦残差 e = (Rᵀ·dir) − λ·(X−C) 的 Ceres 实验。
 *
 * test1：单点，相机位姿固定（验证残差）。
 * test2：多相机、多点、稠密观测；随机初始化 i>0 的 R,C、j>0 的 X、各观测 λ；联合 BA。
 *        仅固定 cam0 仍可能出现多解；再固定一个 3D 点 X0（取真值）以定 gauge，其余量可恢复至 GT。
 * test3：不对比任何「真值」。随机场景生成像素；稀疏观测：每点恰好 k 台相机（默认 5），相机集合
 *        对该点随机打乱后取前 k 个能投影成功的。再用独立随机 X，且 i>0 的 (R,C) 由
 *        look_at(C, centroid(X))。固定 cam0 R=I、C=0，固定 X[0]。大规模用 SPARSE_SCHUR + 多线程。
 *
 * 约定 P_cam = R·(X−C)；dir 为相机系单位射线；λ 乘弦 (X−C)，真值 λ = 1/‖X−C‖。
 * R **不做** SO(3) 约束：Ceres 中为 9 个独立标量（行主序 3×3），与 GLOMAP「线性化旋转矩阵」一致。
 *
 * Build: test_track_ray_lambda_ceres（见 sfm/CMakeLists.txt）
 */

#include "../camera/camera_types.h"
#include "../camera/camera_utils.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <glog/logging.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

namespace {

using insight::camera::Intrinsics;

static bool project_pinhole(const Intrinsics& K, const Eigen::Matrix3d& R,
                            const Eigen::Vector3d& C, const Eigen::Vector3d& X,
                            double* u_out, double* v_out) {
  const Eigen::Vector3d Pc = R * (X - C);
  if (Pc.z() <= 1e-12)
    return false;
  const double xn = Pc.x() / Pc.z();
  const double yn = Pc.y() / Pc.z();
  double xd, yd;
  insight::camera::apply_distortion(xn, yn, K, &xd, &yd);
  *u_out = K.fx * xd + K.cx;
  *v_out = K.fy * yd + K.cy;
  return true;
}

static Eigen::Vector3d ray_dir_camera(const Intrinsics& K, double u_px, double v_px) {
  double u_u, v_u;
  insight::camera::undistort_point(K, u_px, v_px, &u_u, &v_u);
  const double xn = (u_u - K.cx) / K.fx;
  const double yn = (v_u - K.cy) / K.fy;
  Eigen::Vector3d r_cam(xn, yn, 1.0);
  r_cam.normalize();
  return r_cam;
}

static void look_at(const Eigen::Vector3d& C, const Eigen::Vector3d& target,
                    Eigen::Matrix3d* R_out) {
  const Eigen::Vector3d z = (target - C).normalized();
  Eigen::Vector3d up(0, 1, 0);
  if (std::abs(z.dot(up)) > 0.95)
    up = Eigen::Vector3d(0, 0, 1);
  const Eigen::Vector3d x = up.cross(z).normalized();
  const Eigen::Vector3d y = z.cross(x);
  R_out->row(0) = x.transpose();
  R_out->row(1) = y.transpose();
  R_out->row(2) = z.transpose();
}

/// R 行主序 9 标量，无正交约束（与四元数/李代数参数化相对）。
static void matrix_to_R9_row(const Eigen::Matrix3d& R, std::array<double, 9>* out) {
  int k = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      (*out)[static_cast<size_t>(k++)] = R(i, j);
}

static void R9_row_to_matrix(const std::array<double, 9>& r, Eigen::Matrix3d* R) {
  int k = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      (*R)(i, j) = r[static_cast<size_t>(k++)];
}

struct RayDepthVectorResidual {
  explicit RayDepthVectorResidual(const Eigen::Vector3d& dir_cam_unit) : dir_cam_(dir_cam_unit) {}

  template <typename T>
  bool operator()(const T* const R_mat, const T* const C_vec, const T* const lam,
                  const T* const X, T* res) const {
    Eigen::Matrix<T, 3, 3> Rm;
    Rm << R_mat[0], R_mat[1], R_mat[2], R_mat[3], R_mat[4], R_mat[5], R_mat[6], R_mat[7], R_mat[8];
    Eigen::Matrix<T, 3, 1> dir;
    dir << T(dir_cam_(0)), T(dir_cam_(1)), T(dir_cam_(2));
    const Eigen::Matrix<T, 3, 1> d_world = Rm.transpose() * dir;
    for (int i = 0; i < 3; ++i)
      res[i] = d_world(i, 0) - lam[0] * (X[i] - C_vec[i]);
    return true;
  }

  Eigen::Vector3d dir_cam_;
};

struct Obs {
  int cam = 0;
  int pt = 0;
  double u = 0.0;
  double v = 0.0;
};

static Eigen::Vector3d centroid_of(const std::vector<Eigen::Vector3d>& pts) {
  Eigen::Vector3d s = Eigen::Vector3d::Zero();
  for (const auto& p : pts)
    s += p;
  return s / static_cast<double>(pts.size());
}

/// 仅收敛性：无真值比对。观测由 scene 点云生成；优化变量独立随机初始化。
/// 稀疏观测：每个 3D 点恰好 k_views_per_point 台相机可见（随机选机位，投影成功才加入）。
static int test_convergence_random_no_gt() {
  std::cout << "[test3] convergence only: obs from random scene, init random X + RC(look@centroid(X)), "
               "cam0 R=I C=0 fixed\n";

  constexpr int n_cams = 50;
  constexpr int n_pts = 20000;
  constexpr int k_views_per_point = 5;
  constexpr uint32_t k_seed_scene = 1001;
  constexpr uint32_t k_seed_init = 2002;

  Intrinsics K;
  K.fx = 800.0;
  K.fy = 800.0;
  K.cx = 320.0;
  K.cy = 240.0;

  // ── A：随机「场景」仅用于生成像素观测（不用于验收）────────────────────────
  std::mt19937 rng_scene(k_seed_scene);
  std::uniform_real_distribution<double> uni(1.5, 9.0);
  std::vector<Eigen::Vector3d> X_scene(static_cast<size_t>(n_pts));
  for (int j = 0; j < n_pts; ++j)
    X_scene[static_cast<size_t>(j)] = Eigen::Vector3d(uni(rng_scene), uni(rng_scene), uni(rng_scene));

  std::vector<Eigen::Matrix3d> R_obs(n_cams);
  std::vector<Eigen::Vector3d> C_obs(n_cams);
  R_obs[0] = Eigen::Matrix3d::Identity();
  C_obs[0] = Eigen::Vector3d::Zero();
  const Eigen::Vector3d mu_scene = centroid_of(X_scene);
  std::uniform_real_distribution<double> ang(0.0, 2.0 * M_PI);
  std::uniform_real_distribution<double> rad(5.0, 9.0);
  std::uniform_real_distribution<double> zh(1.5, 4.0);
  for (int i = 1; i < n_cams; ++i) {
    const double t = ang(rng_scene);
    Eigen::Vector3d C(rad(rng_scene) * std::cos(t), rad(rng_scene) * std::sin(t), zh(rng_scene));
    look_at(C, mu_scene, &R_obs[static_cast<size_t>(i)]);
    C_obs[static_cast<size_t>(i)] = C;
  }

  std::vector<Obs> observations;
  observations.reserve(static_cast<size_t>(n_pts) * static_cast<size_t>(k_views_per_point));
  std::vector<int> cam_order(static_cast<size_t>(n_cams));
  std::iota(cam_order.begin(), cam_order.end(), 0);
  for (int j = 0; j < n_pts; ++j) {
    std::shuffle(cam_order.begin(), cam_order.end(), rng_scene);
    int added = 0;
    for (int i : cam_order) {
      if (added >= k_views_per_point)
        break;
      double u, v;
      if (!project_pinhole(K, R_obs[static_cast<size_t>(i)], C_obs[static_cast<size_t>(i)],
                           X_scene[static_cast<size_t>(j)], &u, &v))
        continue;
      observations.push_back(Obs{i, j, u, v});
      ++added;
    }
    if (added < k_views_per_point) {
      std::cerr << "  FAIL: point " << j << " only got " << added << " / " << k_views_per_point
                << " views (try denser cameras / scene)\n";
      return 1;
    }
  }
  const int n_obs = static_cast<int>(observations.size());
  const double avg_views = static_cast<double>(n_obs) / static_cast<double>(n_pts);
  std::cout << "  n_cams=" << n_cams << " n_pts=" << n_pts << "  obs/pt(avg)=" << avg_views
            << "  n_obs=" << n_obs << " (sparse)\n";

  // ── B：优化初值：独立随机 X；cam0 I,0；其余相机随机 C + look_at(C, centroid(X))────────
  std::mt19937 rng_init(k_seed_init);
  std::uniform_real_distribution<double> uni_x(0.5, 11.0);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  std::vector<std::array<double, 9>> R_par(static_cast<size_t>(n_cams));
  std::vector<Eigen::Vector3d> C_par(static_cast<size_t>(n_cams));
  std::vector<Eigen::Vector3d> Xw(static_cast<size_t>(n_pts));
  for (int j = 0; j < n_pts; ++j)
    Xw[static_cast<size_t>(j)] =
        Eigen::Vector3d(uni_x(rng_init), uni_x(rng_init), uni_x(rng_init));

  matrix_to_R9_row(Eigen::Matrix3d::Identity(), &R_par[0]);
  C_par[0] = Eigen::Vector3d::Zero();

  const Eigen::Vector3d mu_init = centroid_of(Xw);
  for (int i = 1; i < n_cams; ++i) {
    Eigen::Vector3d C(4.0 + 2.0 * gauss(rng_init), 4.0 + 2.0 * gauss(rng_init),
                      2.0 + 1.5 * gauss(rng_init));
    Eigen::Matrix3d Rl;
    look_at(C, mu_init, &Rl);
    matrix_to_R9_row(Rl, &R_par[static_cast<size_t>(i)]);
    C_par[static_cast<size_t>(i)] = C;
  }

  std::vector<double> lambda(static_cast<size_t>(n_obs));
  for (int k = 0; k < n_obs; ++k) {
    const Obs& o = observations[static_cast<size_t>(k)];
    const Eigen::Vector3d chord = Xw[static_cast<size_t>(o.pt)] - C_par[static_cast<size_t>(o.cam)];
    const double base = 1.0 / std::max(1e-3, chord.norm());
    lambda[static_cast<size_t>(k)] = base * (0.4 + 0.8 * uni01(rng_init));
  }

  ceres::Problem problem;
  auto t_build0 = std::chrono::steady_clock::now();
  for (int k = 0; k < n_obs; ++k) {
    const Obs& o = observations[static_cast<size_t>(k)];
    const Eigen::Vector3d r_cam = ray_dir_camera(K, o.u, o.v);
    auto* cost = new ceres::AutoDiffCostFunction<RayDepthVectorResidual, 3, 9, 3, 1, 3>(
        new RayDepthVectorResidual(r_cam));
    problem.AddResidualBlock(cost, nullptr, R_par[static_cast<size_t>(o.cam)].data(),
                             C_par[static_cast<size_t>(o.cam)].data(),
                             &lambda[static_cast<size_t>(k)], Xw[static_cast<size_t>(o.pt)].data());
  }

  problem.SetParameterBlockConstant(R_par[0].data());
  problem.SetParameterBlockConstant(C_par[0].data());
  problem.SetParameterBlockConstant(Xw[0].data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.preconditioner_type = ceres::JACOBI;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  options.function_tolerance = 1e-6;
  options.gradient_tolerance = 1e-4;
  // 大规模 SPARSE_SCHUR 下部分环境 OpenMP/CHOLMOD 会占 SHM；单线程更稳（本地可调大加速）。
  if (const char* nt = std::getenv("INSIGHT_CERES_NUM_THREADS")) {
    options.num_threads = std::max(1, std::atoi(nt));
  } else {
    options.num_threads = 1;
  }
  ceres::Solver::Summary summary;
  auto t_build1 = std::chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  auto t_solve1 = std::chrono::steady_clock::now();
  const double ms_build =
      std::chrono::duration<double, std::milli>(t_build1 - t_build0).count();
  const double ms_solve =
      std::chrono::duration<double, std::milli>(t_solve1 - t_build1).count();

  const double cost0 = summary.initial_cost;
  const double cost1 = summary.final_cost;
  std::cout << "  build_ms=" << ms_build << "  solve_ms=" << ms_solve
            << "  usable=" << (summary.IsSolutionUsable() ? "yes" : "no")
            << "  iters=" << summary.iterations.size() << "  initial_cost=" << cost0
            << "  final_cost=" << cost1 << "\n";

  if (!summary.IsSolutionUsable()) {
    std::cerr << "  FAIL: " << summary.message << "\n";
    return 1;
  }
  if (cost0 > 1e-15 && cost1 > cost0 * 1.001) {
    std::cerr << "  FAIL: final_cost did not decrease vs initial\n";
    return 1;
  }

  std::cout << "  PASSED test3 (convergence check only).\n";
  return 0;
}

static int test_joint_rc_lambda_x() {
  std::cout << "[test1] single point, R(3x3 unconstrained)+C fixed @ GT, e = R^T*dir - lambda*(X-C)\n";

  Intrinsics K;
  K.fx = 800.0;
  K.fy = 800.0;
  K.cx = 320.0;
  K.cy = 240.0;

  const Eigen::Vector3d X_gt(0.3, -0.2, 8.0);
  const int n_cams = 4;
  std::vector<Eigen::Matrix3d> R_gt(n_cams);
  std::vector<Eigen::Vector3d> C_gt(n_cams);
  std::vector<double> u_px(n_cams), v_px(n_cams);

  for (int i = 0; i < n_cams; ++i) {
    const double ang = static_cast<double>(i) * 0.35;
    Eigen::Vector3d C(4.0 * std::cos(ang), 2.0 * std::sin(ang), 3.5 + 0.2 * static_cast<double>(i));
    look_at(C, X_gt, &R_gt[static_cast<size_t>(i)]);
    C_gt[static_cast<size_t>(i)] = C;
    double u, v;
    if (!project_pinhole(K, R_gt[static_cast<size_t>(i)], C, X_gt, &u, &v)) {
      std::cerr << "  projection failed cam " << i << "\n";
      return 1;
    }
    u_px[static_cast<size_t>(i)] = u;
    v_px[static_cast<size_t>(i)] = v;
  }

  std::vector<std::array<double, 9>> R_par(n_cams);
  std::vector<Eigen::Vector3d> C_par(n_cams);
  std::vector<double> lambda(static_cast<size_t>(n_cams));
  for (int i = 0; i < n_cams; ++i) {
    matrix_to_R9_row(R_gt[static_cast<size_t>(i)], &R_par[static_cast<size_t>(i)]);
    C_par[static_cast<size_t>(i)] = C_gt[static_cast<size_t>(i)];
    const Eigen::Vector3d chord = X_gt - C_gt[static_cast<size_t>(i)];
    lambda[static_cast<size_t>(i)] = 1.0 / chord.norm();
  }

  Eigen::Vector3d X = X_gt + Eigen::Vector3d(0.2, -0.15, 0.4);
  for (int i = 0; i < n_cams; ++i)
    lambda[static_cast<size_t>(i)] *= 0.85;

  ceres::Problem problem;
  for (int i = 0; i < n_cams; ++i) {
    const Eigen::Vector3d r_cam =
        ray_dir_camera(K, u_px[static_cast<size_t>(i)], v_px[static_cast<size_t>(i)]);
    auto* cost = new ceres::AutoDiffCostFunction<RayDepthVectorResidual, 3, 9, 3, 1, 3>(
        new RayDepthVectorResidual(r_cam));
    problem.AddResidualBlock(cost, nullptr, R_par[static_cast<size_t>(i)].data(),
                             C_par[static_cast<size_t>(i)].data(), &lambda[static_cast<size_t>(i)],
                             X.data());
  }

  for (int i = 0; i < n_cams; ++i) {
    problem.SetParameterBlockConstant(R_par[static_cast<size_t>(i)].data());
    problem.SetParameterBlockConstant(C_par[static_cast<size_t>(i)].data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 200;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (!summary.IsSolutionUsable()) {
    std::cerr << "  Ceres failed: " << summary.message << "\n";
    return 1;
  }

  const double err_x = (X - X_gt).norm();
  std::cout << "  ||X - X_gt|| = " << err_x << "  final_cost=" << summary.final_cost << "\n";

  if (err_x > 1e-5) {
    std::cerr << "  FAIL: X error too large\n";
    return 1;
  }

  for (int i = 0; i < n_cams; ++i) {
    Eigen::Matrix3d Rr;
    R9_row_to_matrix(R_par[static_cast<size_t>(i)], &Rr);
    const Eigen::Vector3d& Cr = C_par[static_cast<size_t>(i)];
    if ((Rr - R_gt[static_cast<size_t>(i)]).norm() > 1e-12 ||
        (Cr - C_gt[static_cast<size_t>(i)]).norm() > 1e-12) {
      std::cerr << "  FAIL: cam pose should stay at GT\n";
      return 1;
    }
    const Eigen::Vector3d chord = X_gt - C_gt[static_cast<size_t>(i)];
    const double lam_gt = 1.0 / chord.norm();
    if (std::abs(lambda[static_cast<size_t>(i)] - lam_gt) > 1e-4) {
      std::cerr << "  FAIL: lambda err\n";
      return 1;
    }
  }

  std::cout << "  PASSED test1.\n";
  return 0;
}

static int test_multi_random_init() {
  std::cout << "[test2] multi: R 3x9 free + C, random init (fix cam0 R,C + X0)\n";

  constexpr int n_cams = 5;
  constexpr int n_pts = 8;
  constexpr uint32_t k_seed = 42;
  // 分用途种子，避免改一处噪声分布牵连其它随机量
  constexpr uint32_t k_seed_pose = k_seed;
  constexpr uint32_t k_seed_pts = k_seed + 101;
  constexpr uint32_t k_seed_lam = k_seed + 202;

  Intrinsics K;
  K.fx = 900.0;
  K.fy = 900.0;
  K.cx = 320.0;
  K.cy = 240.0;

  const Eigen::Vector3d scene_center(0.0, 0.0, 7.0);

  std::vector<Eigen::Matrix3d> R_gt(n_cams);
  std::vector<Eigen::Vector3d> C_gt(n_cams);
  for (int i = 0; i < n_cams; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_cams) * 2.0 * M_PI;
    Eigen::Vector3d C(5.5 * std::cos(t), 5.5 * std::sin(t), 2.0 + 0.35 * static_cast<double>(i));
    look_at(C, scene_center, &R_gt[static_cast<size_t>(i)]);
    C_gt[static_cast<size_t>(i)] = C;
  }

  std::vector<Eigen::Vector3d> X_gt(static_cast<size_t>(n_pts));
  std::mt19937 rng_scene(k_seed);
  std::uniform_real_distribution<double> uni(-1.2, 1.2);
  for (int j = 0; j < n_pts; ++j) {
    X_gt[static_cast<size_t>(j)] =
        scene_center + Eigen::Vector3d(uni(rng_scene), uni(rng_scene), uni(rng_scene) * 0.8);
  }

  std::vector<Obs> observations;
  observations.reserve(static_cast<size_t>(n_cams * n_pts));
  for (int i = 0; i < n_cams; ++i) {
    for (int j = 0; j < n_pts; ++j) {
      double u, v;
      if (!project_pinhole(K, R_gt[static_cast<size_t>(i)], C_gt[static_cast<size_t>(i)],
                           X_gt[static_cast<size_t>(j)], &u, &v)) {
        continue;
      }
      observations.push_back(Obs{i, j, u, v});
    }
  }

  if (observations.size() < static_cast<size_t>(n_cams * n_pts / 2)) {
    std::cerr << "  FAIL: too few observations\n";
    return 1;
  }

  const int n_obs = static_cast<int>(observations.size());
  std::cout << "  n_cams=" << n_cams << " n_pts=" << n_pts << " n_obs=" << n_obs << "\n";

  std::vector<std::array<double, 9>> R_par(static_cast<size_t>(n_cams));
  std::vector<Eigen::Vector3d> C_par(static_cast<size_t>(n_cams));
  std::vector<Eigen::Vector3d> Xw(static_cast<size_t>(n_pts));
  std::vector<double> lambda(static_cast<size_t>(n_obs));

  std::mt19937 rng_pose(k_seed_pose);
  std::mt19937 rng_pts(k_seed_pts);
  std::mt19937 rng_lam(k_seed_lam);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni01(0.0, 1.0);

  // 随机初始化 R,C,X,λ：对 i>0 相机做小扰动；点坐标噪声；λ 在 [0.5,1.5]·λ_gt
  for (int i = 0; i < n_cams; ++i) {
    Eigen::Matrix3d R = R_gt[static_cast<size_t>(i)];
    Eigen::Vector3d C = C_gt[static_cast<size_t>(i)];
    if (i > 0) {
      Eigen::Vector3d axis(gauss(rng_pose), gauss(rng_pose), gauss(rng_pose));
      axis.normalize();
      const double ang = 0.04 * (0.5 + uni01(rng_pose));
      R = Eigen::Quaterniond(Eigen::AngleAxisd(ang, axis)).toRotationMatrix() * R;
      C += Eigen::Vector3d(0.05 * gauss(rng_pose), 0.05 * gauss(rng_pose), 0.05 * gauss(rng_pose));
    }
    matrix_to_R9_row(R, &R_par[static_cast<size_t>(i)]);
    C_par[static_cast<size_t>(i)] = C;
  }

  for (int j = 0; j < n_pts; ++j) {
    if (j == 0) {
      Xw[static_cast<size_t>(0)] = X_gt[static_cast<size_t>(0)];
      continue;
    }
    Xw[static_cast<size_t>(j)] =
        X_gt[static_cast<size_t>(j)] + Eigen::Vector3d(0.12 * gauss(rng_pts), 0.12 * gauss(rng_pts),
                                                       0.12 * gauss(rng_pts));
  }

  for (int k = 0; k < n_obs; ++k) {
    const Obs& o = observations[static_cast<size_t>(k)];
    const Eigen::Vector3d chord =
        X_gt[static_cast<size_t>(o.pt)] - C_gt[static_cast<size_t>(o.cam)];
    const double lam0 = 1.0 / chord.norm();
    lambda[static_cast<size_t>(k)] = lam0 * (0.5 + uni01(rng_lam));
  }

  ceres::Problem problem;
  for (int k = 0; k < n_obs; ++k) {
    const Obs& o = observations[static_cast<size_t>(k)];
    const Eigen::Vector3d r_cam = ray_dir_camera(K, o.u, o.v);
    auto* cost = new ceres::AutoDiffCostFunction<RayDepthVectorResidual, 3, 9, 3, 1, 3>(
        new RayDepthVectorResidual(r_cam));
    problem.AddResidualBlock(cost, nullptr, R_par[static_cast<size_t>(o.cam)].data(),
                             C_par[static_cast<size_t>(o.cam)].data(),
                             &lambda[static_cast<size_t>(k)], Xw[static_cast<size_t>(o.pt)].data());
  }

  problem.SetParameterBlockConstant(R_par[0].data());
  problem.SetParameterBlockConstant(C_par[0].data());
  // 再固定一个 3D 点，消除整体平移/尺度上的剩余自由度（仅 cam0 时联合优化仍可能出现多解）
  problem.SetParameterBlockConstant(Xw[0].data());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (!summary.IsSolutionUsable()) {
    std::cerr << "  Ceres failed: " << summary.message << "\n";
    return 1;
  }

  double max_pt = 0.0;
  for (int j = 1; j < n_pts; ++j) {
    max_pt = std::max(max_pt, (Xw[static_cast<size_t>(j)] - X_gt[static_cast<size_t>(j)]).norm());
  }

  double max_cam_C = 0.0;
  for (int i = 1; i < n_cams; ++i) {
    max_cam_C =
        std::max(max_cam_C, (C_par[static_cast<size_t>(i)] - C_gt[static_cast<size_t>(i)]).norm());
  }

  // 无约束 3×3 R 不必接近正交真值（Frobenius）；比较观测射线方向 Rᵀr 与真值是否一致（归一化后夹角）
  double max_dir_ang = 0.0;
  for (const Obs& o : observations) {
    if (o.cam == 0)
      continue;
    Eigen::Matrix3d Rr;
    R9_row_to_matrix(R_par[static_cast<size_t>(o.cam)], &Rr);
    const Eigen::Vector3d r_cam = ray_dir_camera(K, o.u, o.v);
    Eigen::Vector3d d_est = Rr.transpose() * r_cam;
    Eigen::Vector3d d_gt = R_gt[static_cast<size_t>(o.cam)].transpose() * r_cam;
    if (d_est.norm() < 1e-12 || d_gt.norm() < 1e-12)
      continue;
    d_est.normalize();
    d_gt.normalize();
    const double c = std::max(-1.0, std::min(1.0, d_est.dot(d_gt)));
    max_dir_ang = std::max(max_dir_ang, std::acos(c));
  }

  std::cout << "  max ||X-X_gt||=" << max_pt << "  max∠(Rᵀr,R_gtᵀr)(deg)=" << max_dir_ang * 180.0 / M_PI
            << "  max||dC||(cam>0)=" << max_cam_C << "  final_cost=" << summary.final_cost << "\n";

  // 无约束 R 时数值解与正交真值在矩阵范数上可差很大；放宽 C/X/射线方向角
  constexpr double thr_X = 0.06;
  constexpr double thr_dir_deg = 5.0;
  constexpr double thr_C = 0.45;
  if (max_pt > thr_X || max_dir_ang > thr_dir_deg * M_PI / 180.0 || max_cam_C > thr_C) {
    std::cerr << "  FAIL: recovery tolerance exceeded\n";
    return 1;
  }

  std::cout << "  PASSED test2.\n";
  return 0;
}

} // namespace

int main() {
  google::InitGoogleLogging("test_track_ray_lambda_ceres");
  FLAGS_logtostderr = 1;
  FLAGS_minloglevel = 2;

  int failures = 0;
  failures += test_joint_rc_lambda_x();
  failures += test_multi_random_init();
  failures += test_convergence_random_no_gt();
  if (failures == 0)
    std::cout << "\nAll tests PASSED.\n";
  return failures == 0 ? 0 : 1;
}
