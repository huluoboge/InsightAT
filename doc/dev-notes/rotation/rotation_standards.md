# 旋转表示法标准化指南
## 摄影测量 vs 导航领域的欧拉角体系

**版本**: 1.0  
**基础标准**: 
- AIAA R-004-1992 (Aerospace coordinate systems)
- IEEE 1571-2006 (Navigation systems)
- ISPRS (Photogrammetry standards)

---

## 第1部分：基本概念（理论基础）

### 1.1 旋转的两种解释

任何旋转矩阵 **R** 都可以从两个角度理解，它们**互为转置关系**：

#### **主动旋转 (Active Rotation)**
```
定义：固定参考系，向量/刚体旋转
符号：p' = R · p
```
物理含义：
- 参考坐标系保持不动
- 向量或刚体在该坐标系中旋转
- 例：相机相对于固定世界坐标系旋转

#### **被动旋转 (Passive Rotation)**
```
定义：固定向量，参考系旋转
符号：p_new_frame = R · p_old_frame
```
物理含义：
- 向量保持不动，参考坐标系旋转
- 同一个向量在两个坐标系中的表达方式不同
- 例：世界点在世界坐标系和相机坐标系中的坐标

**重要关系**：
```
如果 R_active 描述"主动旋转θ约u轴"
那么 R_passive = R_active^T 描述"坐标系旋转-θ约u轴"
```

**代码中的体现**：
```cpp
// Eigen的AngleAxisd返回的是主动旋转
Mat3 RotationAroundX(double angle) {
    return Eigen::AngleAxisd(angle, Vec3::UnitX()).toRotationMatrix();
    // 这个R代表：向量绕X轴主动旋转angle
}
```

---

### 1.2 欧拉角的两种排列方式

#### **Extrinsic Rotations (固定坐标系/外在旋转)**

旋转依次绕**固定坐标系的轴**应用：

```
R = R_final_axis(γ) · R_middle_axis(β) · R_initial_axis(α)

应用顺序（从右到左）：
1. 先绕初始轴旋转 α
2. 再绕固定的中间轴旋转 β  (注意：不是旋转后的轴！)
3. 最后绕固定的最终轴旋转 γ
```

**特点**：
- 旋转轴始终指向世界坐标系的轴
- 容易可视化和理解
- 从左往右读，矩阵乘法从右往左应用

**例子（Z-Y-X extrinsic）**：
```
R = R_z(γ) · R_y(β) · R_x(α)

物理过程：
1. 绕世界X轴旋转 α
2. 绕世界Y轴旋转 β  (不是旋转后的Y轴)
3. 绕世界Z轴旋转 γ

最终：p_final = R · p_initial
```

#### **Intrinsic Rotations (刚体坐标系/内在旋转)**

旋转依次绕**刚体自身的轴**应用：

```
R_equivalent = R_final_axis(γ) · R_middle_axis(β) · R_initial_axis(α)
[但解释不同]

应用顺序（从右到左）：
1. 先绕刚体初始轴旋转 α，刚体轴也旋转
2. 再绕刚体新的中间轴旋转 β  (刚体的轴)
3. 最后绕刚体最新的轴旋转 γ
```

**特点**：
- 旋转轴随着刚体/载体旋转
- 符合直觉（如飞机的Roll-Pitch-Yaw）
- 数学上等价于extrinsic Z-Y-X，但语义不同

**例子（Z-Y-X intrinsic = X-Y-Z extrinsic）**：
```
Z-Y-X intrinsic (绕刚体轴):
R = R_z(γ) · R_y(β) · R_x(α)

等价于 X-Y-Z extrinsic (绕固定轴):
R = R_x(α) · R_y(β) · R_z(γ)

但旋转顺序和含义完全不同！
```

---

## 第2部分：摄影测量体系 (OmegaPhiKappa)

### 2.1 标准定义（ISPRS标准）

**旋转矩阵形式**：
```
R = R_z(κ) · R_y(φ) · R_x(ω)

其中：
- ω (omega)  = 绕X轴旋转角
- φ (phi)    = 绕Y轴旋转角  
- κ (kappa)  = 绕Z轴旋转角
```

### 2.2 关键特征

| 特性 | 值 |
|------|-----|
| 欧拉角排列 | **Z-Y-X Extrinsic** (固定坐标系) |
| 旋转类型 | **主动旋转** (从载体视角) |
| 基准坐标系 | **世界坐标系** (ECEF或投影坐标) |
| 物理含义 | 相机相对于世界的**方向** |

### 2.3 数学含义

**被动解释（坐标变换）**：
```
p_camera = R · p_world

即：
⎡ x' ⎤     ⎡ p_camera_x ⎤
⎢ y' ⎥ = R ⎢ p_camera_y ⎥
⎣ z' ⎦     ⎣ p_camera_z ⎦

这是一个被动旋转（Passive）：
- 世界中的点坐标 p_world 
- 变换为相机坐标系中的坐标 p_camera
- 点本身不动，坐标系相对于点旋转
```

**关键理解**：
- R本身是**主动旋转矩阵**（数学上，用AngleAxis得到的）
- 但它**应用的场景**是被动旋转（坐标变换）
- 即：R = (world→camera的坐标变换矩阵) = (camera相对world的方向的转置)

### 2.4 Gimbal Lock位置

```
当 φ = ±π/2 时出现gimbal lock

此时前两个轴（X和Y）变为平行，失去一个自由度
```

### 2.5 推导（基于ISPRS）

```
假设：
1. 世界坐标系：地心坐标系或投影坐标系
2. 相机坐标系：X=右, Y=下, Z=前（图像坐标系约定）

则欧拉角顺序为：
1. ω: 绕世界X轴旋转（控制上下方向）
2. φ: 绕世界Y轴旋转（控制左右方向）
3. κ: 绕世界Z轴旋转（控制旋转方向）

最终矩阵：R_world→camera = R_z(κ) · R_y(φ) · R_x(ω)
```

---

## 第3部分：导航领域欧拉角（Yaw-Pitch-Roll / NED）

### 3.1 标准定义（AIAA R-004-1992, IEEE 1571-2006）

**旋转矩阵形式**：
```
两种等价写法：

写法A（Body-fixed/Intrinsic）：
R_body-fixed = R_z(ψ) · R_y(θ) · R_x(φ)
[绕刚体的轴]

写法B（Equivalent Extrinsic）：
R_equivalent = R_x(φ) · R_y(θ) · R_z(ψ)
[绕固定的轴，顺序相反]

其中：
- φ (phi)   = Roll   (绕X轴，左右滚动)
- θ (theta) = Pitch  (绕Y轴，上下俯仰)
- ψ (psi)   = Yaw    (绕Z轴，左右转向)
```

### 3.2 关键特征

| 特性 | 值 |
|------|-----|
| 欧拉角排列 | **Z-Y-X Intrinsic** (刚体坐标系) |
| 等价extrinsic | **X-Y-Z Extrinsic** (固定坐标系) |
| 旋转类型 | **主动旋转** (刚体旋转) |
| 基准坐标系 | **刚体坐标系** (Body-Fixed) |
| 物理含义 | 刚体/无人机相对于世界的**姿态** |

### 3.3 数学含义

**两种物理解释**：

#### **解释A：World→Body (变换矩阵)**
```
R_world→body = R_z(ψ) · R_y(θ) · R_x(φ)

p_body = R_world→body · p_world

含义：世界中的点坐标变换为刚体坐标系中的坐标
这是被动旋转（Passive）
```

#### **解释B：Body→World (方向矩阵)**
```
R_body→world = R_world→body^T = R_x(-φ) · R_y(-θ) · R_z(-ψ)

p_world = R_body→world · p_body

含义：刚体坐标系中的向量旋转到世界坐标系
这是主动旋转（Active）
```

**关键**：
- 这两个解释是**互为转置**的
- AIAA标准中，通常定义 R = R_world→body（被动旋转）
- 但有些惯导系统定义 R = R_body→world（主动旋转）
- **必须显式说明！**

### 3.4 Gimbal Lock位置

```
当 θ = ±π/2 时出现gimbal lock

此时Y轴旋转导致X和Z轴变为平行
```

### 3.5 坐标系约定（NED vs ENU）

#### **NED坐标系 (North-East-Down)** - 导航标准
```
X = North (北)
Y = East  (东)
Z = Down  (向下)

右手系，常用于INS/GNSS

在NED中的欧拉角：
- Roll  (φ)  : 绕North轴，左右翼倾斜
- Pitch (θ)  : 绕East轴，机头上下
- Yaw   (ψ)  : 绕Down轴，机头左右

最常见的Yaw-Pitch-Roll定义
```

#### **ENU坐标系 (East-North-Up)** - 遥感/GIS常用
```
X = East  (东)
Y = North (北)
Z = Up    (向上)

右手系，常用于遥感反演

在ENU中的欧拉角：
- Roll  (φ)  : 绕East轴
- Pitch (θ)  : 绕North轴
- Yaw   (ψ)  : 绕Up轴

与NED的Z-Y-X顺序对应关系复杂
```

---

## 第4部分：摄影测量 vs 导航对比

### 4.1 完整对比表

| 维度 | 摄影测量(OPK) | 导航(YPR/NED) |
|------|------------------|------------------|
| **欧拉角顺序** | Z-Y-X Extrinsic | Z-Y-X Intrinsic |
| **等价Extrinsic** | Z-Y-X Extrinsic | X-Y-Z Extrinsic |
| **旋转解释** | Fixed axes | Body-fixed axes |
| **基准坐标系** | 世界(ECEF/投影) | 刚体(Body-fixed) |
| **物理含义** | 相机指向 | 刚体姿态 |
| **矩阵形式** | R = R_z(κ)R_y(φ)R_x(ω) | R = R_z(ψ)R_y(θ)R_x(φ) |
| **应用** | p_camera = R·p_world | p_body = R·p_world (或 p_world=R^T·p_body) |
| **Gimbal Lock** | φ = ±π/2 | θ = ±π/2 |
| **角度含义** | ω控制俯仰，φ控制横滚，κ控制航向 | φ控制横滚，θ控制俯仰，ψ控制航向 |
| **旋转类型** | 主动(刚体视角) → 被动(应用时) | 主动(刚体视角) → 被动(应用时) |

### 4.2 数学等价性和区别

**表面相似**：
```
都是Z-Y-X旋转组合
都可以用相同的矩阵乘法表示
```

**本质区别**：
```
OmegaPhiKappa:
- 绕"固定世界坐标系"的轴旋转
- 从"相机固定，世界旋转"的被动视角
- 坐标变换的目的

YawPitchRoll (Intrinsic):
- 绕"随刚体旋转"的轴旋转  
- 从"刚体旋转，世界固定"的主动视角
- 刚体姿态描述的目的

即使使用相同的数值，语义完全不同！
```

---

## 第5部分：标准化建议（针对InsightAT）

### 5.1 当前代码状态分析

**你的 DBPose 结构**：
```cpp
struct DBPose {
    double x, y, z;           // 世界坐标系中的相机位置
    double omega, phi, kappa; // 欧拉角
    // ... 其他字段
};
```

**需要澄清的问题**：

1. **坐标系约定**：
   ```
   当前：(x,y,z) 是什么坐标系？
   - 投影坐标系？ECEF？ENU？
   
   应该明确标注
   ```

2. **旋转矩阵方向**：
   ```
   当前代码：R = R_z(κ) · R_y(φ) · R_x(ω)
   
   问题：这个R是什么？
   - 世界→相机？(被动旋转)
   - 还是相机→世界？(主动旋转)
   
   应该有明确的数学定义
   ```

3. **angle单位和坐标系**：
   ```
   当前字段：
   int angleUnit;    // 0=度，1=弧度
   int coordinate;   // 0=x-right,y-down,z-forward; 1=x-right,y-up,z-backward
   int eulerAngle;   // 0=OmegaPhiKappa, 1=PhiOmegaKappa
   
   这些字段缺乏标准意义上的定义
   ```

### 5.2 标准化方案（四步）

#### **步骤1：明确坐标系**
```cpp
enum class WorldCoordinateSystem {
    ECEF,              // 地心地固坐标系
    ProjectedUTM,      // 投影坐标系(UTM)
    Local_ENU,         // 本地东北天
    Local_NED,         // 本地北东下
};

enum class CameraCoordinateSystem {
    RightDownForward,  // X右，Y下，Z前（图像坐标约定）
    RightUpBackward,   // X右，Y上，Z后（OpenCV约定）
};
```

#### **步骤2：明确旋转矩阵方向**
```cpp
enum class RotationMatrixType {
    World_to_Camera,   // 被动旋转：p_camera = R · p_world
    Camera_to_World,   // 主动旋转：p_world = R · p_camera
};
```

#### **步骤3：改进DBPose**
```cpp
struct StandardizedDBPose {
    // 位置
    WorldCoordinateSystem position_coord_system;
    double x, y, z;  // 世界坐标系
    
    // 旋转
    RotationMatrixType rotation_type;  // 明确指定矩阵含义
    double omega, phi, kappa;          // 欧拉角（总是弧度）
    
    // Quaternion (冗余存储，避免gimbal lock)
    double qx, qy, qz, qw;
    
    // 元数据
    EulerAngleConvention convention;   // OmegaPhiKappa 或 YawPitchRoll
    CameraCoordinateSystem camera_coord_system;
    
    // 文档（注释）
    /* 
    标准定义（ISPRS摄影测量）：
    - 坐标系：World=投影坐标或ECEF, Camera=RightDownForward
    - 旋转矩阵：R = R_z(κ) · R_y(φ) · R_x(ω)
    - 含义：p_camera = R · p_world (World → Camera, 被动旋转)
    - 单位：所有角度以弧度存储
    */
};
```

#### **步骤4：创建转换/验证函数**
```cpp
// 验证旋转矩阵的合法性
bool isValidRotationMatrix(const Mat3& R);

// 显式转换（带方向标注）
Mat3 eulerToRotationMatrix(
    double omega, double phi, double kappa,
    RotationMatrixType type);

Vec3 rotationMatrixToEuler(
    const Mat3& R,
    RotationMatrixType type);

// 四元数（避免gimbal lock）
Quaternion eulerToQuaternion(...);
Vec3 quaternionToEuler(...);
```

---

## 第6部分：参考文献

### 权威标准
1. **AIAA R-004-1992**：Aerospace Coordinate Systems  
   - 最权威的航空坐标系定义
   - 规定了Body-fixed欧拉角

2. **IEEE 1571-2006**：Navigation Systems - Definitions  
   - 导航系统标准
   - NED/ENU坐标系定义

3. **ISPRS Commission I/2**：Photogrammetry Standards
   - 摄影测量的OPK约定
   - 坐标变换标准

4. **Goldstein, H.**："Classical Mechanics"
   - 欧拉角的数学基础

5. **Shoemake, K. (1985)**："Animating Rotation with Quaternion Curves"
   - Extrinsic vs Intrinsic的经典解释

### 在线资源
- https://en.wikipedia.org/wiki/Euler_angles （基础概念）
- ISPRS标准文档（摄影测量约定）
- AIAA标准购买版本

---

## 总结

| 关键点 | 说明 |
|--------|------|
| **主动 vs 被动** | 需要同时考虑"矩阵本身"和"应用场景" |
| **摄影测量(OPK)** | Z-Y-X Extrinsic，用于World→Camera坐标变换 |
| **导航(YPR)** | Z-Y-X Intrinsic，用于描述Body的姿态 |
| **Gimbal Lock** | OPK在φ=±π/2，YPR在θ=±π/2 |
| **标准化** | 每个项目都必须显式定义坐标系和旋转矩阵方向 |
| **四元数** | 优先用于计算，避免gimbal lock和数值问题 |

---

**下一步**：基于此标准框架实现符合规范的代码
