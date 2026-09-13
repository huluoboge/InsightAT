# 旋转表示法标准化 - 文档索引

本目录包含 InsightAT 中**旋转表示（欧拉角、四元数等）的正式标准与快速参考**。与 Agent 协作的过程记录（代码分析、项目总结等）位于 [`doc/dev-notes/imp/rotation/`](../../../dev-notes/imp/rotation/)。

## 📚 文档列表

### 1. **standards.md** ⭐ 核心参考文件
**用途**：理解旋转表示的标准和理论  
**内容**：主动/被动旋转、Extrinsic/Intrinsic 欧拉角、摄影测量 (OmegaPhiKappa) 与导航 (YawPitchRoll/NED) 标准、两体系对比、标准化建议。

**位置**: [standards.md](standards.md)

---

### 2. **quick-reference.md** ⚡ 快速参考
**用途**：日常开发快速查阅  
**内容**：一页纸速查表、常见错误及修正、转换工具用法、调试技巧、数据结构对比。

**位置**: [quick-reference.md](quick-reference.md)

---

## 🗺️ 快速导航

- **新加入 / 想快速了解**：先读 [quick-reference.md](quick-reference.md)，再读 [standards.md](standards.md) 第 1 部分。
- **需要修改旋转相关代码**：参考 [standards.md](standards.md) 第 2/3 部分（具体约定），并查 [quick-reference.md](quick-reference.md) 的常见错误。
- **代码问题分析与改进方案**：见 [`imp-rotation-code-analysis.md`](../../../dev-notes/imp/rotation/imp-rotation-code-analysis.md)；阶段总结见 [`ROTATION_PROJECT_SUMMARY.md`](../../../dev-notes/archive/ROTATION_PROJECT_SUMMARY.md)。

---

## 与 design 的对应

设计文档中的坐标系与旋转约定见 [08_coordinate_and_rotation.md](../08_coordinate_and_rotation.md)。
