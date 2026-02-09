# 坐标系配置对话框 - 实现总结

## 完成内容

### 1. 修改数据结构 (`src/database/database_types.h`)

**变更内容：**
- ✅ 删除 `RotationConvention::kNone` 枚举值
- ✅ 重新编号为 `kOmegaPhiKappa = 0`（摄影测量）和 `kYawPitchRoll = 1`（航空学）
- ✅ 更新 `CoordinateSystem` 默认值：`rotation_convention = RotationConvention::kOmegaPhiKappa`
- ✅ 在 `database_types.cpp` 中移除对 `kNone` 的引用

### 2. 创建核心 Widget (`src/ui/widgets/CoordinateSystemConfigWidget.*`)

**核心功能：**
- ✅ **四种坐标系类型支持**
  - `LOCAL`: 本地坐标系（无需配置）
  - `EPSG`: EPSG 代码（可通过对话框选择或手动编辑）
  - `ENU`: East-North-Up 本地坐标系（带参考点和可选原点）
  - `WKT`: OGC WKT 定义（可通过对话框选择或手动编辑）

- ✅ **动态 UI 切换**
  - 使用 `QStackedWidget` 根据选中的坐标系类型动态切换内容区

- ✅ **实时表单验证**
  - 监听所有输入组件的变化信号
  - 动态验证表单有效性
  - 发出 `validationChanged(bool)` 信号控制 OK 按钮状态
  - 实时显示验证错误提示

- ✅ **EPSG/WKT 对话框集成**
  - Browse 按钮打开 `SpatialReferenceDialog`
  - 用户可选择坐标系后自动填充编辑框
  - 用户可继续编辑已选择的内容

- ✅ **RotationConvention 选择**
  - 两个 RadioButton：摄影测量（默认）和航空学
  - 默认选中摄影测量规约

### 3. 创建对话框容器 (`src/ui/dialogs/CoordinateSystemConfigDialog.*`)

**职责：**
- ✅ 对话框框架和 OK/Cancel 按钮管理
- ✅ 连接 Widget 的验证信号到 OK 按钮启用状态
- ✅ 初始状态禁用 OK 按钮（直到表单有效）
- ✅ 返回用户配置的 `CoordinateSystem` 对象

### 4. 集成到应用 (`src/ui/MainWindow.cpp`)

**集成点：**
- ✅ 包含新的对话框头文件
- ✅ 实现 `onSetCoordinateSystem()` 方法
- ✅ 菜单项连接正确
- ✅ 通过 `ProjectDocument::updateCoordinateSystem()` 更新项目数据

### 5. 编译配置 (`src/ui/CMakeLists.txt`)

- ✅ 注册 `CoordinateSystemConfigWidget.h/cpp`
- ✅ 注册 `CoordinateSystemConfigDialog.h/cpp`
- ✅ 编译成功，所有文件正确链接

## 功能特性详解

### LOCAL 模式
- **UI**: 显示说明文本"No additional configuration required..."
- **验证**: 始终有效
- **数据**: 无需设置 definition, reference, origin

### EPSG 模式
- **UI**: 只读文本框显示已选 EPSG 代码 + Browse 按钮
- **Browse 按钮**: 打开 SpatialReferenceDialog，用户可选择或搜索坐标系
- **编辑**: 选择后显示 EPSG 代码（如 "EPSG:4326"），用户可继续修改
- **验证**: 检查 definition 非空
- **错误提示**: 在下方显示"EPSG code is required"

### ENU 模式
- **Reference Point** (必需)
  - Latitude: [-90, 90]°（WGS84）
  - Longitude: [-180, 180]°（WGS84）
  - Altitude: 任意范围 (米)
- **Local Origin** (可选)
  - X, Y, Z: 支持科学记数法，范围 [-1e9, 1e9]
  - 不输入时默认为 (0, 0, 0)，不保存到 origin
- **验证**: 纬度和经度范围检查
- **错误提示**: 显示具体范围错误

### WKT 模式
- **UI**: QPlainTextEdit 编辑框 + Browse 按钮
- **Browse 按钮**: 打开 SpatialReferenceDialog，返回 WKT 字符串自动填充
- **编辑**: 用户可继续修改或粘贴自定义 WKT
- **验证**: 检查包含 "PROJCS" 或 "GEOGCS" 关键字
- **错误提示**: 显示 WKT 格式要求

### RotationConvention
- **两选项**（无 None）
  - 摄影测量 (ω, φ, κ)：默认
  - 航空学 (Y, P, R)
- **设置**: 加载坐标系时自动恢复用户之前的选择

## 使用流程

### 用户创建新项目
1. 菜单 → Edit → "Set Coordinate System..."
2. 选择坐标系类型（LOCAL/EPSG/ENU/WKT）
3. 输入相应参数
4. OK 按钮自动启用（当表单有效时）
5. 点击 OK 保存配置

### 用户编辑现有项目的坐标系
1. 菜单 → Edit → "Set Coordinate System..."
2. 对话框自动加载当前坐标系配置
3. 修改参数
4. 点击 OK 保存

### EPSG/WKT 数据库选择
1. 在 EPSG 或 WKT 模式下点击 Browse 按钮
2. SpatialReferenceDialog 打开
3. 搜索或浏览坐标系
4. 选中后自动填充编辑框
5. 可继续编辑（如截取 EPSG 代码、微调 WKT）
6. 返回主对话框，点击 OK 保存

## 代码架构

```
CoordinateSystemConfigDialog (对话框框架)
    ├── OK/Cancel 按钮
    └── CoordinateSystemConfigWidget (核心 Widget)
        ├── 坐标系类型选择 (QRadioButton ×4)
        ├── 动态内容区 (QStackedWidget)
        │   ├── Local 页 (empty)
        │   ├── EPSG 页 (QLineEdit + Browse Button)
        │   ├── ENU 页 (6× QDoubleSpinBox)
        │   └── WKT 页 (QPlainTextEdit + Browse Button)
        ├── RotationConvention 选择 (QRadioButton ×2)
        └── 验证和信号逻辑
            ├── validationChanged(bool) → OK 按钮启用状态
            └── 实时错误提示 (QLabel 下方显示)
```

## 验证规则

| 模式 | 必需字段 | 范围/格式 | 错误提示 |
|------|--------|---------|--------|
| LOCAL | 无 | - | - |
| EPSG | definition | 非空 | "EPSG code is required" |
| ENU | lat, lon, alt | lat∈[-90,90], lon∈[-180,180] | 具体范围错误 |
| WKT | definition | 含"PROJCS"或"GEOGCS" | "WKT must contain 'PROJCS' or 'GEOGCS'" |

## 编译信息

- **编译状态**: ✅ 成功
- **可执行文件**: `/build/InsightAT_New` (817 KB)
- **库文件**: `libInsightATUI.a` (已生成)
- **警告**: 仅有未使用参数的警告（不影响功能）

## 后续可扩展性

1. **国际化**: 使用 `tr()` 封装所有字符串
2. **高级验证**: 可在 validateEPSGMode() 等方法中添加网络验证
3. **缓存**: 可缓存用户最近使用的坐标系
4. **预设**: 可添加常用坐标系的快速选择按钮
5. **坐标系转换**: 可集成 GDAL 进行即时坐标转换验证

---

**实现日期**: 2026-02-09  
**主要特性**: 实时验证、动态 UI、多种坐标系支持、集成式对话框选择
