# InsideAT SfM 系统优化设计文档

**主题：Local BA / Track Compression / BA 调度策略**

作者：小萝卜
面向系统：InsideAT SfM / 大规模倾斜摄影 / 长 Track 场景

---

# 1 设计思想（Design Philosophy）

在大规模 SfM / 倾斜摄影场景中，系统性能瓶颈通常不是特征提取或匹配，而是 **Bundle Adjustment (BA)**。

BA 的计算复杂度主要取决于：

```
observations = Σ(track length)
```

而不是：

```
points 数量
```

在航测或倾斜摄影数据中，经常出现：

```
avg track length > 30
max track length > 50
```

这会导致：

```
observations 数量爆炸
```

例如：

```
points = 1M
avg track = 50
observations = 50M
```

但事实上：

* 几何约束通常只需要 **6~10 个 view**
* 大量观测来自 **同一航线**
* view direction 高度相关

因此：

```
50 observations ≈ 10 observations 信息量
```

**核心设计思想**

1. **控制 BA 规模**
2. **减少冗余 observation**
3. **保证 view direction 覆盖**
4. **Local BA 代替 Global BA**

最终目标：

```
BA规模 ≈ constant
```

---

# 2 系统架构（System Architecture）

InsideAT 推荐的 BA 系统结构：

```
Incremental SfM
      │
      ▼
New Camera Registration
      │
      ▼
Local BA Scheduler
      │
      ▼
Observation Compression
      │
      ▼
Local Bundle Adjustment
      │
      ▼
Periodic Global BA
```

模块说明：

| 模块                      | 作用          |
| ----------------------- | ----------- |
| Registration            | 新相机位姿估计     |
| Local BA Scheduler      | 选择参与 BA 的相机 |
| Observation Compression | 压缩 track    |
| Local BA                | 局部优化        |
| Global BA               | 周期性全局修正     |

---

# 3 Local BA 策略

## 3.1 相机分层结构

Local BA 中相机分为三类：

```
NEW CAMERA
    │
    ▼
ACTIVE CAMERAS (优化)
    │
    ▼
FIXED CAMERAS (约束)
```

结构示意：

```
        FIXED cameras
              ▲
              │
        ACTIVE cameras
              ▲
              │
          NEW camera
```

---

## 3.2 相机选择算法

### Step 1 找 connected cameras

根据 track graph 找：

```
C1 = cameras sharing tracks with new camera
```

通常：

```
|C1| = 40 ~ 150
```

---

### Step 2 计算相关性

评分：

```
score(cam_i) =
    shared_tracks(cam_i, new_cam)
```

---

### Step 3 选 ACTIVE cameras

```
ACTIVE = Top K cameras
```

推荐：

```
K = 20
```

---

### Step 4 选 FIXED cameras

```
FIXED = next M cameras
```

推荐：

```
M = 30
```

FIXED cameras 只提供约束，不优化。

---

# 4 三维点选择策略

Local BA 只使用：

```
points seen by ACTIVE cameras
```

过滤条件：

```
track_length >= 3
```

限制最大数量：

```
max_points = 80k
```

如果超过：

```
随机采样
或
优先保留长 track
```

评分建议：

```
score(point) =
    track_length
  + triangulation_angle
```

---

# 5 Observation Compression（核心优化）

## 5.1 问题

在倾斜摄影数据中：

```
track length > 50
```

很多 observation：

```
view direction 几乎相同
```

例如：

```
---- flight line ----
o o o o o o o o o o
      *
     point
```

这些观测提供的约束高度相关。

---

## 5.2 压缩目标

对每个 point：

```
保留 K 个最有价值的 observation
```

推荐：

```
K = 8 ~ 12
```

---

## 5.3 Angular Coverage Selection

计算：

```
v_i = normalize(camera_center_i - point)
```

Greedy 算法：

```
1 选 baseline 最大的 pair
2 while |selected| < K
3   选择 view
4   最大化 min angle
```

目标：

```
view direction 均匀分布
```

---

## 5.4 压缩效果

例：

```
points = 1M
avg track = 50
```

压缩后：

```
track = 10
observations = 10M
```

结果：

```
BA速度提升 ≈ 3~5x
```

精度变化：

```
< 0.2 px
```

---

# 6 Track 管理策略

## 6.1 BA Track 限制

```
max_track_length_BA = 10
```

但：

```
PnP track = full
```

这样：

```
BA 快
PnP 稳
```

---

## 6.2 Track 动态控制

如果：

```
track_length > 12
```

策略：

```
替换最差 observation
```

保持：

```
track ≈ 10
```

---

# 7 Local BA 调度策略

不要每张图都做 BA。

推荐：

```
每 5 张图做一次 Local BA
```

或：

```
shared_tracks(new_cam) > threshold
```

---

# 8 BA 规模控制

推荐规模：

```
ACTIVE cameras = 20
FIXED cameras  = 30

points ≤ 80k

track length ≤ 10
```

BA 规模：

```
observations ≈ 300k ~ 500k
```

这个规模 BA 会非常稳定。

---

# 9 Global BA 策略

Local BA 会产生累积误差。

周期性做：

```
Global BA
```

推荐：

```
every 50~100 images
```

但不需要全部点：

```
random 10% points
```

这样：

```
全局误差被修正
```

但计算仍然可控。

---

# 10 InsideAT 推荐参数

最终推荐配置：

```
ACTIVE cameras = 20
FIXED cameras  = 30

max_points_local_BA = 80k

max_track_length_BA = 10

Local BA interval = 5 images

Global BA interval = 50~100 images
```

预计效果：

```
BA时间 ↓ 5~10x
```

同时保持：

```
几何精度基本不变
```

---

# 11 关键结论

在大规模 SfM 中：

```
性能瓶颈 ≠ points
```

而是：

```
observations
```

因此系统设计的核心是：

```
控制 observation 数量
```

主要手段：

```
Local BA
Track Compression
Observation Selection
BA Scheduling
```

这些策略组合后，可以显著提升系统效率。

---

**文档结束**
