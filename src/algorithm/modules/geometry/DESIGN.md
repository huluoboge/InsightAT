# Two-View Geometry Estimation Module Design

> **版本**: 1.0  
> **创建日期**: 2026-02-12  
> **目标**: 定义 InsightAT 双视图几何估计模块的架构、算法实现和自校准策略

---

## 1. 核心设计理念

### 1.1 混合几何估计策略（F → E → Calibration）

**核心思想：** 先用鲁棒的 F 矩阵（不依赖内参）建立几何关系，再通过三角化验证质量，最后优化内参实现自校准。

```
阶段 A: 全局鲁棒估计
  F 矩阵 (8点算法 + RANSAC) → 内点集合 + 对极约束

阶段 B: 内参敏感估计
  E 矩阵 (5点算法 + RANSAC) → R, t 位姿 + 三角化

阶段 C: 退化检测与自校准
  三角化质量检查 → 焦距/畸变优化 → 更新内参
```

**为什么不直接估计 E 矩阵？**
- **内参初值不准确**：用户提供的焦距可能是 EXIF 估计或默认值
- **F 矩阵更鲁棒**：仅依赖像素坐标，对内参错误免疫
- **渐进式改进**：F → E 提供更稳定的初始化

### 1.2 函数式设计原则

**所有几何估计器都是纯函数：**
```cpp
namespace insight::algorithm::geometry {

// 输入：匹配点坐标，输出：F 矩阵 + 内点索引
FundamentalResult estimateFundamental(
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    const RansacOptions& options = RansacOptions()
);

// 输入：归一化坐标，输出：E 矩阵 + 位姿
EssentialResult estimateEssential(
    const std::vector<Eigen::Vector2d>& points1_normalized,
    const std::vector<Eigen::Vector2d>& points2_normalized,
    const RansacOptions& options = RansacOptions()
);

// 输入：E 矩阵 + 归一化坐标，输出：R, t + 3D 点
TriangulationResult recoverPoseAndTriangulate(
    const Eigen::Matrix3d& E,
    const std::vector<Eigen::Vector2d>& points1_normalized,
    const std::vector<Eigen::Vector2d>& points2_normalized,
    const std::vector<size_t>& inlier_indices
);

}  // namespace insight::algorithm::geometry
```

---

## 2. 算法实现

### 2.1 Fundamental Matrix（基础矩阵）

**8 点算法 + RANSAC**

**数学基础：**
对极约束：`x2^T F x1 = 0`，其中 `F` 为 3×3 矩阵，`rank(F) = 2`

**归一化 8 点算法（Hartley-Zisserman）：**
```cpp
Eigen::Matrix3d compute8Point(
    const std::vector<Eigen::Vector2d>& pts1,
    const std::vector<Eigen::Vector2d>& pts2
) {
    // 1. 归一化坐标（平移到原点，缩放到 sqrt(2)）
    auto [pts1_norm, T1] = normalizePoints(pts1);
    auto [pts2_norm, T2] = normalizePoints(pts2);
    
    // 2. 构建线性方程组 Af = 0
    Eigen::MatrixXd A(pts1.size(), 9);
    for (size_t i = 0; i < pts1.size(); ++i) {
        double x1 = pts1_norm[i].x(), y1 = pts1_norm[i].y();
        double x2 = pts2_norm[i].x(), y2 = pts2_norm[i].y();
        A.row(i) << x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1;
    }
    
    // 3. SVD 求解（最小奇异值对应的右奇异向量）
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd f = svd.matrixV().col(8);
    Eigen::Matrix3d F = Eigen::Map<Eigen::Matrix3d>(f.data()).transpose();
    
    // 4. 强制 rank(F) = 2
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_F(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singular_values = svd_F.singularValues();
    singular_values(2) = 0;  // 最小奇异值置零
    F = svd_F.matrixU() * singular_values.asDiagonal() * svd_F.matrixV().transpose();
    
    // 5. 反归一化
    return T2.transpose() * F * T1;
}
```

**Sampson 距离（RANSAC 误差度量）：**
```cpp
double sampsonDistance(const Eigen::Vector2d& x1, 
                       const Eigen::Vector2d& x2,
                       const Eigen::Matrix3d& F) {
    Eigen::Vector3d x1_h(x1.x(), x1.y(), 1.0);
    Eigen::Vector3d x2_h(x2.x(), x2.y(), 1.0);
    
    Eigen::Vector3d Fx1 = F * x1_h;
    Eigen::Vector3d Ftx2 = F.transpose() * x2_h;
    
    double numerator = (x2_h.transpose() * F * x1_h)(0, 0);
    double denominator = Fx1(0)*Fx1(0) + Fx1(1)*Fx1(1) + 
                         Ftx2(0)*Ftx2(0) + Ftx2(1)*Ftx2(1);
    
    return (numerator * numerator) / denominator;
}
```

**RANSAC 参数：**
- **最小样本数**: 8 点
- **误差阈值**: 3.0 像素（Sampson 距离）
- **置信度**: 0.9999（99.99% 概率找到正确模型）
- **最大迭代次数**: 自适应（基于内点比例）

**参考实现：** PoseLib（推荐）或自实现

### 2.2 Essential Matrix（本质矩阵）

**5 点算法 + RANSAC**

**数学基础：**
对极约束：`x2_norm^T E x1_norm = 0`，其中 `E = [t]_× R`

**关键性质：**
- `E` 有两个相等的奇异值，第三个为零
- `E` 可分解为 4 种可能的 (R, t) 组合

**实现策略：**
```cpp
// 使用 PoseLib 的 5 点算法（Nistér/Stewénius）
#include "PoseLib/robust/estimators.h"

EssentialResult estimateEssential(
    const std::vector<Eigen::Vector2d>& pts1_norm,
    const std::vector<Eigen::Vector2d>& pts2_norm,
    const RansacOptions& options
) {
    // 1. RANSAC 循环
    poselib::RansacOptions poselib_opts;
    poselib_opts.max_iterations = options.max_iterations;
    poselib_opts.success_prob = options.confidence;
    poselib_opts.threshold = options.threshold;
    
    poselib::CameraPose pose;
    poselib::RansacStats stats;
    
    int num_inliers = poselib::estimate_relative_pose(
        pts1_norm, pts2_norm,
        poselib_opts,
        &pose,
        &stats
    );
    
    // 2. 提取 E 矩阵
    Eigen::Matrix3d E = pose.E();
    
    // 3. 返回结果
    return {E, pose.R(), pose.t(), stats.inliers};
}
```

**为什么使用 PoseLib？**
- ✅ 生产级质量（COLMAP 使用）
- ✅ 优化的 5 点算法实现
- ✅ 支持多种 RANSAC 变体（MAGSAC++, LO-RANSAC）
- ✅ Header-only，易集成

**备选方案：** OpenCV `cv::findEssentialMat`（违反低层级控制原则，但可用于验证）

### 2.3 Pose Recovery & Triangulation（位姿恢复与三角化）

**从 E 矩阵恢复位姿：**

```cpp
TriangulationResult recoverPoseAndTriangulate(
    const Eigen::Matrix3d& E,
    const std::vector<Eigen::Vector2d>& pts1_norm,
    const std::vector<Eigen::Vector2d>& pts2_norm,
    const std::vector<size_t>& inlier_indices
) {
    // 1. SVD 分解 E = U Σ V^T
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    
    // 2. 构建 4 种候选位姿
    Eigen::Matrix3d W;
    W << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;
    
    std::vector<CameraPose> candidates = {
        {U * W * V.transpose(), U.col(2)},
        {U * W * V.transpose(), -U.col(2)},
        {U * W.transpose() * V.transpose(), U.col(2)},
        {U * W.transpose() * V.transpose(), -U.col(2)}
    };
    
    // 3. 检查每个候选：三角化并统计前方点数量
    CameraPose best_pose;
    std::vector<Eigen::Vector3d> best_points3D;
    int max_front_count = 0;
    
    for (const auto& candidate : candidates) {
        auto points3D = triangulateDLT(pts1_norm, pts2_norm, 
                                       Eigen::Matrix3d::Identity(), candidate.R,
                                       Eigen::Vector3d::Zero(), candidate.t);
        
        int front_count = countPointsInFront(points3D, candidate);
        if (front_count > max_front_count) {
            max_front_count = front_count;
            best_pose = candidate;
            best_points3D = points3D;
        }
    }
    
    return {best_pose.R, best_pose.t, best_points3D};
}
```

**DLT 三角化（Direct Linear Transform）：**
```cpp
Eigen::Vector3d triangulateDLT(
    const Eigen::Vector2d& x1_norm,
    const Eigen::Vector2d& x2_norm,
    const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2,
    const Eigen::Vector3d& t1, const Eigen::Vector3d& t2
) {
    // 投影矩阵 P = K [R | t]，归一化坐标下 K = I
    Eigen::Matrix<double, 3, 4> P1, P2;
    P1 << R1, t1;
    P2 << R2, t2;
    
    // 构建线性方程组 AX = 0
    Eigen::Matrix4d A;
    A.row(0) = x1_norm.x() * P1.row(2) - P1.row(0);
    A.row(1) = x1_norm.y() * P1.row(2) - P1.row(1);
    A.row(2) = x2_norm.x() * P2.row(2) - P2.row(0);
    A.row(3) = x2_norm.y() * P2.row(2) - P2.row(1);
    
    // SVD 求解
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X_h = svd.matrixV().col(3);
    
    return X_h.head<3>() / X_h(3);  // 齐次坐标转欧几里得
}
```

---

## 3. 退化检测（Degeneracy Detection）

### 3.1 检测标准

**目标：** 避免使用质量差的图像对进行内参估计

**检测维度：**

| 检测项 | 阈值 | 原因 |
|--------|------|------|
| **前方交会角** | < 5° | 三角化精度差（深度不确定性高） |
| **基线/深度比** | < 0.01 | 近似纯旋转（无法恢复尺度） |
| **重投影误差** | > 2.0 px | 模型拟合质量差 |
| **内点比例** | < 30% | 匹配质量差或场景退化 |
| **前方点比例** | < 80% | 位姿估计错误 |

**实现：**
```cpp
struct DegeneracyFlags {
    bool sufficient_baseline = false;
    bool sufficient_parallax = false;
    bool low_reprojection_error = false;
    bool high_inlier_ratio = false;
    bool high_front_ratio = false;
    
    bool isNonDegenerate() const {
        return sufficient_baseline && sufficient_parallax && 
               low_reprojection_error && high_inlier_ratio && high_front_ratio;
    }
};

DegeneracyFlags checkDegeneracy(
    const std::vector<Eigen::Vector3d>& points3D,
    const CameraPose& pose1,
    const CameraPose& pose2,
    const MatchResult& matches,
    const std::vector<size_t>& inliers
) {
    DegeneracyFlags flags;
    
    // 1. 前方交会角
    double median_parallax = computeMedianParallax(points3D, pose1, pose2);
    flags.sufficient_parallax = (median_parallax > 5.0);
    
    // 2. 基线长度
    double baseline = (pose2.t - pose1.t).norm();
    double median_depth = computeMedianDepth(points3D, pose1);
    flags.sufficient_baseline = (baseline / median_depth > 0.01);
    
    // 3. 重投影误差
    double rms_error = computeReprojectionRMS(points3D, matches, inliers, pose1, pose2);
    flags.low_reprojection_error = (rms_error < 2.0);
    
    // 4. 内点比例
    double inlier_ratio = static_cast<double>(inliers.size()) / matches.num_matches;
    flags.high_inlier_ratio = (inlier_ratio > 0.3);
    
    // 5. 前方点比例
    int front_count = countPointsInFront(points3D, pose1, pose2);
    double front_ratio = static_cast<double>(front_count) / points3D.size();
    flags.high_front_ratio = (front_ratio > 0.8);
    
    return flags;
}
```

### 3.2 平面场景检测（Homography vs Fundamental）

**问题：** 平面场景或纯旋转时，F 矩阵退化

**检测方法：**
```cpp
bool isPlanarScene(const MatchResult& matches) {
    // 同时估计 H 和 F，比较内点数量
    auto H_result = estimateHomography(matches);
    auto F_result = estimateFundamental(matches);
    
    double ratio = static_cast<double>(H_result.num_inliers) / F_result.num_inliers;
    return (ratio > 0.9);  // H 内点数接近 F，说明场景平面
}
```

**处理策略：**
- 平面场景：使用 H 矩阵分解（4 种解）
- 非平面场景：使用 F → E 流程

---

## 4. 自动内参估计（Auto-Calibration）

### 4.1 优先队列策略

**目标：** 为每个 `camera_id` 找到最优图像对进行内参估计

**优先级评分：**
```cpp
double computeCalibrationScore(const TwoViewGeometry& geometry) {
    double score = 0.0;
    
    // 1. 内点数量（权重：40%）
    score += 0.4 * (geometry.num_inliers / 10000.0);
    
    // 2. 前方交会角（权重：30%，15° 为理想值）
    double angle_score = std::min(geometry.median_parallax / 15.0, 1.0);
    score += 0.3 * angle_score;
    
    // 3. 基线长度（权重：20%）
    double baseline_score = std::min(geometry.baseline_ratio / 0.1, 1.0);
    score += 0.2 * baseline_score;
    
    // 4. 重投影误差（权重：10%，越小越好）
    double error_score = 1.0 - std::min(geometry.reprojection_rms / 2.0, 1.0);
    score += 0.1 * error_score;
    
    return score;
}
```

**优先队列实现：**
```cpp
struct CalibrationCandidate {
    std::string image1_id, image2_id;
    int camera_id;
    double score;
    
    bool operator<(const CalibrationCandidate& other) const {
        return score < other.score;  // 最大堆
    }
};

std::priority_queue<CalibrationCandidate> buildCalibrationQueue(
    const MatchGraph& graph
) {
    std::priority_queue<CalibrationCandidate> queue;
    
    for (const auto& edge : graph.edges) {
        if (!edge.degeneracy_flags.isNonDegenerate()) continue;
        
        int camera_id = graph.nodes[edge.image1_id].camera_id;
        double score = computeCalibrationScore(edge.geometry);
        
        queue.push({edge.image1_id, edge.image2_id, camera_id, score});
    }
    
    return queue;
}
```

### 4.2 焦距与畸变优化

**从三角化结果反向优化内参：**

```cpp
struct IntrinsicsRefinement {
    double fx, fy;  // 焦距
    double k1, k2;  // 径向畸变
};

IntrinsicsRefinement refineIntrinsics(
    const std::vector<Eigen::Vector3d>& points3D,
    const std::vector<Eigen::Vector2d>& observations1,  // 像素坐标
    const std::vector<Eigen::Vector2d>& observations2,
    const CameraPose& pose1,
    const CameraPose& pose2,
    const Eigen::Vector2d& principal_point,
    double fx_init, double fy_init
) {
    // 使用 Ceres Solver 优化
    ceres::Problem problem;
    
    // 优化变量
    double fx = fx_init, fy = fy_init;
    double k1 = 0.0, k2 = 0.0;
    
    // 添加残差
    for (size_t i = 0; i < points3D.size(); ++i) {
        // 视图1的重投影误差
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 1, 1, 1, 1>(
                new ReprojectionCost(points3D[i], observations1[i], pose1, principal_point)
            ),
            nullptr,
            &fx, &fy, &k1, &k2
        );
        
        // 视图2的重投影误差
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 1, 1, 1, 1>(
                new ReprojectionCost(points3D[i], observations2[i], pose2, principal_point)
            ),
            nullptr,
            &fx, &fy, &k1, &k2
        );
    }
    
    // 求解
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    return {fx, fy, k1, k2};
}
```

**重投影代价函数：**
```cpp
struct ReprojectionCost {
    Eigen::Vector3d point3D;
    Eigen::Vector2d observed;
    CameraPose pose;
    Eigen::Vector2d principal_point;
    
    template <typename T>
    bool operator()(const T* const fx, const T* const fy,
                    const T* const k1, const T* const k2,
                    T* residuals) const {
        // 1. 世界坐标 → 相机坐标
        Eigen::Matrix<T, 3, 1> p_cam = pose.R.cast<T>() * point3D.cast<T>() + pose.t.cast<T>();
        
        // 2. 归一化坐标
        T x_norm = p_cam[0] / p_cam[2];
        T y_norm = p_cam[1] / p_cam[2];
        
        // 3. 畸变校正
        T r2 = x_norm * x_norm + y_norm * y_norm;
        T distortion = T(1.0) + k1[0] * r2 + k2[0] * r2 * r2;
        T x_dist = x_norm * distortion;
        T y_dist = y_norm * distortion;
        
        // 4. 投影到像素
        T u = fx[0] * x_dist + T(principal_point.x());
        T v = fy[0] * y_dist + T(principal_point.y());
        
        // 5. 残差
        residuals[0] = u - T(observed.x());
        residuals[1] = v - T(observed.y());
        
        return true;
    }
};
```

---

## 5. 工具链设计

### 5.1 isat_build_graph

**输入：** 所有 `.isat_match` 文件  
**输出：** `.isat_graph.json`（全局匹配图）

**流程：**
```cpp
int main(int argc, char* argv[]) {
    // 1. 扫描匹配文件目录
    auto match_files = findAllMatchFiles(matches_dir);
    
    // 2. 并行处理：对每对执行 F 矩阵估计
    MatchGraph graph;
    for (const auto& match_file : match_files) {
        auto match_data = loadMatchIDC(match_file);
        
        // F 矩阵 RANSAC
        auto F_result = estimateFundamental(
            match_data.coords_pixel.col(0, 1),  // 图像1坐标
            match_data.coords_pixel.col(2, 3),  // 图像2坐标
            ransac_options
        );
        
        // 记录边信息
        graph.addEdge({
            match_data.image1_id,
            match_data.image2_id,
            match_file,
            match_data.num_matches,
            F_result.num_inliers,
            computeGeometricScore(F_result),
            false  // is_calibration_candidate（稍后更新）
        });
    }
    
    // 3. 标记高质量边（用于内参估计）
    graph.markCalibrationCandidates(
        /*min_inliers=*/500,
        /*min_geometric_score=*/0.7
    );
    
    // 4. 保存 JSON
    saveMatchGraph(graph, output_json);
    
    return 0;
}
```

### 5.2 isat_calibrate

**输入：** `.isat_graph.json`  
**输出：** `.isat_cameras_calibrated.json`

**流程：**
```cpp
int main(int argc, char* argv[]) {
    // 1. 加载匹配图
    MatchGraph graph = loadMatchGraph(graph_json);
    
    // 2. 构建优先队列
    auto calib_queue = buildCalibrationQueue(graph);
    
    // 3. 为每个相机估计内参
    std::map<int, CameraIntrinsics> calibrated_cameras;
    
    while (!calib_queue.empty()) {
        auto candidate = calib_queue.top();
        calib_queue.pop();
        
        // 如果该相机已校准，跳过
        if (calibrated_cameras.count(candidate.camera_id)) continue;
        
        // 加载匹配数据
        auto match_data = loadMatchIDC(graph.getEdge(candidate.image1_id, candidate.image2_id).match_file);
        
        // F → E → 三角化
        auto F_result = estimateFundamental(match_data);
        auto E = fundamentalToEssential(F_result.F, initial_intrinsics);
        auto triangulation = recoverPoseAndTriangulate(E, match_data, F_result.inliers);
        
        // 退化检测
        auto degeneracy = checkDegeneracy(triangulation, match_data, F_result);
        if (!degeneracy.isNonDegenerate()) {
            LOG(WARNING) << "Pair " << candidate.image1_id << "-" << candidate.image2_id 
                        << " is degenerate, skipping";
            continue;
        }
        
        // 优化内参
        auto refined_intrinsics = refineIntrinsics(
            triangulation.points3D,
            match_data.observations1,
            match_data.observations2,
            triangulation.pose1,
            triangulation.pose2,
            initial_intrinsics
        );
        
        // 保存结果
        calibrated_cameras[candidate.camera_id] = refined_intrinsics;
        LOG(INFO) << "Camera " << candidate.camera_id << " calibrated: "
                  << "fx=" << refined_intrinsics.fx << ", fy=" << refined_intrinsics.fy;
    }
    
    // 4. 保存校准结果
    saveCalibratedCameras(calibrated_cameras, output_json);
    
    return 0;
}
```

---

## 6. CUDA RANSAC 路线图（Phase 3）

### 6.1 动机

**CPU RANSAC 瓶颈：**
- F 矩阵：每次迭代需要 8 点 + SVD（~100 µs）
- 典型迭代次数：1000-10000 次
- 总耗时：0.1-1 秒/图像对
- 批量处理 10000 对：数小时

**CUDA 加速潜力：**
- 并行采样：同时生成 10000 组候选模型
- 批量评估：并行计算所有匹配点的误差
- 预期加速：10-100x

### 6.2 实现策略

**参考：** SiftGPU 的 GLSL shader 实现（`SiftMatch.cu`）

**CUDA 核函数设计：**
```cuda
__global__ void ransacFundamentalKernel(
    const float2* points1,      // [N] 图像1坐标
    const float2* points2,      // [N] 图像2坐标
    int num_points,
    int num_iterations,
    float threshold,
    uint32_t* random_seeds,     // [num_iterations] 随机种子
    float* F_matrices,          // [num_iterations * 9] 输出
    int* inlier_counts          // [num_iterations] 输出
) {
    int iter = blockIdx.x * blockDim.x + threadIdx.x;
    if (iter >= num_iterations) return;
    
    // 1. 随机采样 8 点
    uint32_t seed = random_seeds[iter];
    int samples[8];
    randomSample(samples, num_points, seed);
    
    // 2. 计算 F 矩阵（8点算法）
    float F[9];
    compute8PointGPU(points1, points2, samples, F);
    
    // 3. 统计内点数量
    int inliers = 0;
    for (int i = 0; i < num_points; ++i) {
        float error = sampsonDistanceGPU(points1[i], points2[i], F);
        if (error < threshold) inliers++;
    }
    
    // 4. 写回结果
    for (int i = 0; i < 9; ++i) {
        F_matrices[iter * 9 + i] = F[i];
    }
    inlier_counts[iter] = inliers;
}
```

**主机端代码：**
```cpp
FundamentalResult estimateFundamentalCUDA(
    const std::vector<Eigen::Vector2f>& points1,
    const std::vector<Eigen::Vector2f>& points2
) {
    // 1. 上传数据到 GPU
    thrust::device_vector<float2> d_points1 = convertToFloat2(points1);
    thrust::device_vector<float2> d_points2 = convertToFloat2(points2);
    
    // 2. 生成随机种子
    thrust::device_vector<uint32_t> d_seeds(num_iterations);
    generateRandomSeeds(d_seeds);
    
    // 3. 执行 RANSAC
    thrust::device_vector<float> d_F_matrices(num_iterations * 9);
    thrust::device_vector<int> d_inlier_counts(num_iterations);
    
    int block_size = 256;
    int grid_size = (num_iterations + block_size - 1) / block_size;
    ransacFundamentalKernel<<<grid_size, block_size>>>(
        thrust::raw_pointer_cast(d_points1.data()),
        thrust::raw_pointer_cast(d_points2.data()),
        points1.size(),
        num_iterations,
        threshold,
        thrust::raw_pointer_cast(d_seeds.data()),
        thrust::raw_pointer_cast(d_F_matrices.data()),
        thrust::raw_pointer_cast(d_inlier_counts.data())
    );
    
    // 4. 找到最优模型
    auto max_iter = thrust::max_element(d_inlier_counts.begin(), d_inlier_counts.end());
    int best_iter = max_iter - d_inlier_counts.begin();
    
    // 5. 下载结果
    std::vector<float> F_data(9);
    thrust::copy(d_F_matrices.begin() + best_iter * 9,
                 d_F_matrices.begin() + (best_iter + 1) * 9,
                 F_data.begin());
    
    Eigen::Matrix3f F = Eigen::Map<Eigen::Matrix3f>(F_data.data());
    return {F.cast<double>(), /* inliers */};
}
```

**挑战：**
- GPU 上的 SVD 实现（可使用 cuBLAS/cuSOLVER）
- 内存管理（避免频繁 CPU ↔ GPU 传输）
- 调试困难度

---

## 7. 第三方库集成

### 7.1 PoseLib

**下载与集成：**
```bash
# 1. 下载源码
cd /home/recon/Git/02jones/InsightAT/third_party
git clone https://github.com/vlarsson/PoseLib.git
cd PoseLib
git checkout <stable_commit_hash>

# 2. 记录版本
echo "PoseLib commit: $(git rev-parse HEAD)" > ../poselib_version.txt

# 3. CMake 集成
```

**third_party/CMakeLists.txt 添加：**
```cmake
# PoseLib
add_subdirectory(PoseLib)
```

**src/algorithm/modules/geometry/CMakeLists.txt：**
```cmake
target_link_libraries(geometry_module
    PUBLIC
        Eigen3::Eigen
        PoseLib::PoseLib
)
```

### 7.2 Ceres Solver（已有）

**用于：**
- Bundle Adjustment（全局优化）
- 内参优化（本模块）
- 位姿图优化

---

## 8. 测试策略

### 8.1 单元测试

```cpp
TEST(FundamentalEstimator, SyntheticData) {
    // 1. 生成合成数据
    Eigen::Matrix3d F_true = generateRandomFundamental();
    auto [pts1, pts2] = generateCorrespondences(F_true, 1000, /*noise=*/0.5);
    
    // 2. 添加外点
    addOutliers(pts1, pts2, 200);
    
    // 3. 估计 F 矩阵
    auto result = estimateFundamental(pts1, pts2);
    
    // 4. 验证
    EXPECT_NEAR(computeMatrixDistance(result.F, F_true), 0.0, 1e-2);
    EXPECT_GT(result.num_inliers, 900);
}
```

### 8.2 集成测试

**使用真实数据：**
```bash
# 1. 运行完整流程
./isat_match -i pairs.json -o matches/
./isat_build_graph -m matches/ -o graph.json
./isat_calibrate -g graph.json -o cameras.json

# 2. 验证内参估计
python3 scripts/evaluate_calibration.py cameras.json ground_truth.json
```

---

## 9. 实施检查清单

### Phase 1: 基础实现（当前）
- [ ] `geometry/fundamental.h/cpp` - F 矩阵估计
- [ ] `geometry/essential.h/cpp` - E 矩阵估计
- [ ] `geometry/triangulation.h/cpp` - DLT 三角化
- [ ] `geometry/degeneracy.h/cpp` - 退化检测
- [ ] 集成 PoseLib
- [ ] 单元测试：合成数据

### Phase 2: 工具链
- [ ] `tools/isat_build_graph.cpp` - 全局图构建
- [ ] `tools/isat_calibrate.cpp` - 自校准
- [ ] `geometry/calibration.h/cpp` - 内参优化
- [ ] 集成测试：真实数据

### Phase 3: CUDA 加速
- [ ] `geometry/ransac_cuda.cu` - CUDA RANSAC
- [ ] Thrust/cuBLAS 集成
- [ ] 性能 benchmark
- [ ] 精度对比（CUDA vs CPU）

---

**最后更新**: 2026-02-12  
**审阅状态**: 待实现
