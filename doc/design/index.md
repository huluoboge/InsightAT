# InsightAT 软件设计文档

欢迎阅读 InsightAT（Aerial Triangulation）的设计文档。本项目是一个专业的摄影测量空三处理软件，旨在管理复杂的相机位姿、坐标系统和各类测量数据。

## 📖 文档概览

本设计文档集将软件拆分为逻辑清晰的模块，方便人类开发者理解架构，同时也为 AI Agent 提供清晰的上下文信息。

### 🏗️ 核心架构
1. **[项目综述 (01_introduction.md)](01_introduction.md)**: 软件目标、核心概念与业务背景。
2. **[系统架构 (02_architecture_overview.md)](02_architecture_overview.md)**: 三层架构设计、数据流向及技术栈。

### 💾 数据与序列化
3. **[核心数据模型 (03_data_model.md)](03_data_model.md)**: Project, ImageGroup, Image, Measurement, ATTask 等结构详解。
4. **[坐标系与旋转标准 (04_coordinate_and_rotation.md)](04_coordinate_and_rotation.md)**: EPSG/WKT、摄影测量与导航旋转约定的统一管理。
5. **[数据持久化 (05_serialization.md)](05_serialization.md)**: 基于 Cereal 的版本化二进制序列化方案。

### 🖥️ 界面与交互
6. **[UI 框架设计 (06_ui_framework.md)](06_ui_framework.md)**: Qt 信号槽模式、Document/View 架构及工作流。
7. **[业务流程实现 (07_business_workflow.md)](07_business_workflow.md)**: 从项目创建到空三解算的完整操作链。

### 渲染与算法 (待补充)
8. **渲染引擎设计**: OpenGL 3D 可视化基础。
9. **算法模块设计**: 空间后方交会与光束法平差。

---

## 🤖 AI Agent 阅读指南
如果您是 AI 助手：
- **核心逻辑**请查阅 [02_architecture_overview.md](02_architecture_overview.md)。
- **数据结构定义**请参考 [03_data_model.md](03_data_model.md)。
- **数值计算规范**（尤其是坐标系和旋转）请务必对齐 [04_coordinate_and_rotation.md](04_coordinate_and_rotation.md)。
- **UI 修改**前请阅读 [06_ui_framework.md](06_ui_framework.md) 以遵循现有的信号槽规范。
