# 旋转表示法标准化 - 文档索引

本目录包含关于 InsightAT 中旋转表示（欧拉角、四元数等）的完整标准化文档。

## 📚 文档列表

### 1. **ROTATION_STANDARDS.md** ⭐ 核心参考文件
**用途**：理解旋转表示的标准和理论
**内容**：
- 主动/被动旋转的定义
- Extrinsic/Intrinsic 欧拉角的区别
- 摄影测量 (OmegaPhiKappa) 标准
- 导航领域 (YawPitchRoll/NED) 标准
- 两个体系的对比
- 标准化建议

**适用于**：
- 理解基础概念的开发者
- 参与旋转工具库实现的人员
- 进行代码审查的人

**位置**: [doc/rotation/ROTATION_STANDARDS.md](ROTATION_STANDARDS.md)

---

### 2. **CODE_ANALYSIS_ROTATION.md** 🔍 问题分析文件
**用途**：了解现有代码的问题和改进方案
**内容**：
- 当前 DBPose 结构的分析
- 7个关键问题的详细说明（带影响分析）
- 每个问题的标准做法
- 短期/中期/长期改进方案
- 代码示例和建议

**适用于**：
- 需要修改旋转相关代码的开发者
- 进行代码审查和架构升级的人员
- 想了解为什么现有代码有问题的人

**位置**: [doc/rotation/CODE_ANALYSIS_ROTATION.md](CODE_ANALYSIS_ROTATION.md)

---

### 3. **ROTATION_QUICK_REFERENCE.md** ⚡ 快速参考
**用途**：日常开发中快速查阅
**内容**：
- 一页纸速查表
- 常见错误及修正
- 转换工具使用方法
- 调试技巧
- 数据结构对比

**适用于**：
- 日常编码和调试
- 快速找答案的开发者
- 新手学习

**位置**: [doc/rotation/ROTATION_QUICK_REFERENCE.md](ROTATION_QUICK_REFERENCE.md)

---

### 4. **ROTATION_PROJECT_SUMMARY.md** 📋 项目总结
**用途**：了解整个项目的进展和规划
**内容**：
- 阶段1（理论学习）的完成情况
- 关键发现总结
- 标准化清单
- 下一步计划（P0/P1/P2）
- 文档使用指南

**适用于**：
- 项目管理人员
- 想全面了解进展的人
- 确定自己该参与哪一部分的人

**位置**: [doc/rotation/ROTATION_PROJECT_SUMMARY.md](ROTATION_PROJECT_SUMMARY.md)

---

## 🗺️ 快速导航

### 我是...

#### 新加入项目的开发者
1. 先读 ROTATION_QUICK_REFERENCE.md（5分钟）
2. 再读 ROTATION_STANDARDS.md 第1部分（10分钟）
3. 然后查看代码中的注释（已更新）

#### 需要修改旋转相关代码的人
1. 读 CODE_ANALYSIS_ROTATION.md 第5部分（改进方案）
2. 参考 ROTATION_STANDARDS.md 第2或3部分（具体约定）
3. 检查 ROTATION_QUICK_REFERENCE.md 的常见错误

#### 进行代码审查的人
1. 用 CODE_ANALYSIS_ROTATION.md 第5部分的清单检查代码
2. 验证是否符合 ROTATION_STANDARDS.md 中的定义
3. 参考 ROTATION_QUICK_REFERENCE.md 的检查列表

#### 想理解为什么要这么做的人
1. 读 ROTATION_STANDARDS.md 第4部分（摄影测量 vs 导航对比）
2. 再读 CODE_ANALYSIS_ROTATION.md 的问题描述
3. 了解为什么现有代码有问题

---

## 📊 文档覆盖范围

| 内容 | ROTATION_STANDARDS | CODE_ANALYSIS | QUICK_REFERENCE | SUMMARY |
|------|-------------------|---------------|-----------------|---------|
| **概念解释** | ✅ 详细 | ❌ | ✅ 简洁 | ❌ |
| **代码问题** | ❌ | ✅ 深度 | ✅ 常见 | ✅ 概览 |
| **改进方案** | ✅ 框架 | ✅ 详细 | ✅ 代码 | ✅ 规划 |
| **快速查阅** | ❌ | ❌ | ✅ | ❌ |
| **学习资料** | ✅ | ✅ | ✅ | ✅ |

---

## 🚀 实现路线图

### 阶段1（已完成 ✅）
- [x] 理论学习（ROTATION_STANDARDS.md）
- [x] 代码分析（CODE_ANALYSIS_ROTATION.md）
- [x] 文档编写（4个文件）

### 阶段2（待进行）
- [ ] 实现 rotation_utils.h/cpp
- [ ] 创建单元测试 test_rotation_utils.cpp
- [ ] 验证与标准的一致性

### 阶段3（待进行）
- [ ] 增强 DBPose 结构
- [ ] 升级文件格式到 Version 3 (JSON)
- [ ] 与其他系统（COLMAP等）对接

---

## 📖 标准文献

以下国际标准在本文档中被引用和解释：

1. **AIAA R-004-1992** - Aerospace Coordinate Systems
2. **IEEE 1571-2006** - Navigation Systems Standards
3. **ISPRS Photogrammetry Standards**

这些标准的详细说明见 ROTATION_STANDARDS.md 第6部分。

---

## 💬 常见问题

### Q: 我只想快速了解，应该读哪个文件？
**A**: ROTATION_QUICK_REFERENCE.md（5-10分钟）

### Q: 代码中有旋转相关的bug，从哪开始？
**A**: CODE_ANALYSIS_ROTATION.md → 找到对应问题 → 查看改进方案

### Q: 我想完全理解这个主题，应该读什么顺序？
**A**: 
1. ROTATION_STANDARDS.md 第1-4部分
2. CODE_ANALYSIS_ROTATION.md 全文
3. ROTATION_QUICK_REFERENCE.md
4. ROTATION_PROJECT_SUMMARY.md

### Q: 何时应该使用四元数而不是欧拉角？
**A**: 当 φ 接近 ±π/2（gimbal lock风险）时。详见 ROTATION_QUICK_REFERENCE.md。

---

## 📞 贡献指南

如果你发现文档中的问题或想改进它们：

1. 确保改进符合引用的标准
2. 更新所有相关文件
3. 保持各文件之间的一致性
4. 更新本 README 中的索引

---

**最后更新**: 2026-02-08  
**维护者**: InsightAT 旋转标准化项目  
**版本**: 1.0 (阶段1完成)
