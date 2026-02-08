# 旋转表示法 - 快速参考卡片

## 一页纸速查表

### InsightAT 当前使用：摄影测量 (OmegaPhiKappa)

```
矩阵公式：
  R = R_z(κ) · R_y(φ) · R_x(ω)

应用方式：
  p_camera = R · p_world

含义：
  ✓ World → Camera 坐标变换（被动旋转）
  ✓ 绕固定世界坐标系的轴旋转（Extrinsic）
  ✓ Z-Y-X 旋转顺序

角度范围：
  ω (omega):  [-π, π]  绕X轴
  φ (phi):    [-π/2, π/2]  绕Y轴  ⚠️ gimbal lock at ±π/2
  κ (kappa):  [-π, π]  绕Z轴

单位：弧度（内存）/ 度（文件，待标准化）

参考系统：
  位置 (x,y,z): 投影坐标或ECEF（当前未明确）
  相机坐标系：X=右, Y=下, Z=前
```

---

### 快速检查列表：使用欧拉角时

- [ ] 确认已用弧度不是度数？
- [ ] 确认 φ 不接近 ±π/2？（gimbal lock）
- [ ] 确认旋转矩阵方向是 World→Camera？
- [ ] 如果φ接近±π/2，用四元数代替？

---

### 常见错误

#### ❌ 错误1：混淆坐标系
```cpp
// 错误
double x, y, z;  // 哪个坐标系？投影？ECEF？

// 正确
WorldCoordinateSystem coord_system = WorldCoordinateSystem::ProjectedUTM;
double x, y, z;  // 现在清楚了
```

#### ❌ 错误2：反向旋转
```cpp
// 错误
p_world = R * p_camera;  // 反了！

// 正确
p_camera = R * p_world;  // World → Camera
```

#### ❌ 错误3：度数和弧度混淆
```cpp
// 错误
double omega = 10;  // 10度还是10弧度？

// 正确
double omega_rad = degreesToRadians(10);  // 明确说明
```

#### ❌ 错误4：Gimbal lock处理不当
```cpp
// 错误
if (phi == M_PI/2) {
    // ...  只检查精确等于
}

// 正确
if (std::abs(phi - M_PI/2) < THRESHOLD) {
    LOG(WARNING) << "Gimbal lock risk!";
    // 使用四元数
}
```

---

### 标准文献速查

| 需要了解 | 参考文件 |
|---------|---------|
| **基本概念** | ROTATION_STANDARDS.md 第1部分 |
| **OmegaPhiKappa详解** | ROTATION_STANDARDS.md 第2部分 |
| **与导航系统的区别** | ROTATION_STANDARDS.md 第3、4部分 |
| **代码中的问题** | CODE_ANALYSIS_ROTATION.md |
| **如何改进** | CODE_ANALYSIS_ROTATION.md 第5部分 |
| **全局概览** | ROTATION_PROJECT_SUMMARY.md |

---

### 转换工具（等待实现）

```cpp
// 当这些函数实现后，使用这些：

// 欧拉角 → 旋转矩阵
Mat3 R = eulerToRotationMatrix(omega, phi, kappa);

// 欧拉角 → 四元数（推荐，避免gimbal lock）
Quaternion q = eulerToQuaternion(omega, phi, kappa);

// 四元数 → 旋转矩阵
Mat3 R = q.toRotationMatrix();

// 验证旋转矩阵合法性
if (!isValidRotationMatrix(R)) {
    LOG(ERROR) << "Invalid rotation matrix!";
}
```

---

### 数据结构（目标）

```cpp
// 当前（不完整）
struct DBPose {
    double x, y, z;
    double omega, phi, kappa;
    int angleUnit;      // 0=deg, 1=rad
    int coordinate;     // 0=右下前, 1=右上后
    int eulerAngle;     // 0=OPK, 1=PhiOmegaKappa
};

// 目标（完整）
struct StandardizedDBPose {
    // 坐标
    WorldCoordinateSystem position_coord_system;  // ← 新
    double x, y, z;
    
    // 旋转方式
    RotationMatrixType rotation_type;  // ← 新，明确说明World→Camera
    double omega, phi, kappa;
    
    // 替代表达
    double qx, qy, qz, qw;  // ← 新，四元数，避免gimbal lock
    
    // GPS权重
    float weight_x, weight_y, weight_z;
};
```

---

### 如何调试旋转问题

```cpp
// 步骤1：验证矩阵
Mat3 R = eulerToRotationMatrix(omega, phi, kappa);
if (!isValidRotationMatrix(R)) {
    // det(R) ≠ 1 或 R·R^T ≠ I
    LOG(ERROR) << "Rotation matrix is invalid!";
}

// 步骤2：检测gimbal lock
if (std::abs(phi - M_PI/2) < 0.1 || std::abs(phi + M_PI/2) < 0.1) {
    LOG(WARNING) << "Near gimbal lock, consider quaternion";
}

// 步骤3：验证方向（单位测试）
Vec3 world_point(1, 0, 0);
Vec3 camera_point = R * world_point;
// 检查结果是否符合预期

// 步骤4：与已知系统对比
// 输出相同的数据到COLMAP或Metashape
// 验证结果一致
```

---

### 记住

| 要素 | 值 |
|------|-----|
| **默认单位** | 弧度 |
| **默认旋转方向** | World → Camera |
| **默认欧拉角顺序** | Z-Y-X (Extrinsic) |
| **Gimbal lock风险** | φ ≈ ±90° |
| **建议替代方案** | 四元数 |
| **文档位置** | src/Common/ROTATION_*.md |

---

**何时查看完整文档**：
- 😕 搞不清楚一个概念 → ROTATION_STANDARDS.md
- 🐛 代码有问题 → CODE_ANALYSIS_ROTATION.md
- 📊 想要全局了解 → ROTATION_PROJECT_SUMMARY.md
- ⚡ 需要快速答案 → 这个文件！
