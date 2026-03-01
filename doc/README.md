# InsightAT 文档目录

本目录包含 InsightAT 项目的所有文档，按功能模块组织。

## 📁 目录结构

```
doc/
├── README.md                    ← 你在这里
├── design/                      ← 正式设计文档（架构、数据模型、规范）
│   └── rotation/                ← 旋转表示法标准化文档
├── dev-notes/                   ← 开发过程记录（实现思考、修复报告、与 Agent 协作稿，非最终规范）
└── tools/                       ← 工具使用说明、快速上手、测试记录
```

## 🎯 快速导航

### 开发笔记（dev-notes）

存放**实现过程中的记录与思考**（含与 Agent 协作写下的报告、修复总结、迁移计划等）。内容为**过程稿**，最终以 `design/` 与代码为准。

**入口**: [dev-notes/README.md](dev-notes/README.md)

---

### 旋转表示法（Rotation Representation）

InsightAT 的摄影测量模块使用欧拉角和四元数表示相机姿态。这涉及复杂的坐标系转换和约定，已被完整标准化。

**入口**: [design/rotation/rotation_readme.md](design/rotation/rotation_readme.md)

**包含内容**：
- ✅ 国际标准的完整解释（AIAA, IEEE, ISPRS）
- ✅ 摄影测量 vs 导航体系的对比
- ✅ 快速参考卡片和常见错误
- 问题分析、改进方案、项目总结等过程记录在 [dev-notes/](dev-notes/)

**适合读者**：
- 需要理解相机姿态表示的开发者
- 进行 DBPose 相关代码修改的人
- 与外部摄影测量软件对接的人

---

## 📚 文档贡献指南

如果你想添加或更新文档：

1. **确定分类** - 选择合适的子目录（如 `rotation/`）
2. **遵循格式** - 参考已有文档的结构和风格
3. **更新导航** - 在对应子目录的 README 中更新索引
4. **更新本文件** - 如果添加了新的子目录，更新这个总 README
5. **链接代码** - 在代码注释中链接相关文档

## 🔗 代码中的文档引用

代码中的注释应该链接到相应的文档。例如：

```cpp
/**
 * 相关文档：doc/design/rotation/rotation_standards.md
 */
struct DBPose {
    // ...
};
```

这样可以帮助开发者快速找到所需的文档。

---

## 📖 现有文档列表

### design/ - 设计文档（权威参考）

见 [design/index.md](design/index.md)。

### dev-notes/ - 开发笔记（过程记录）

实现/修复/迁移过程记录，详见 [dev-notes/README.md](dev-notes/README.md)。以 design 与代码为准。

### rotation/ 子目录（旋转表示法）

| 文件 | 用途 | 读者 | 时间 |
|------|------|------|------|
| [rotation_readme.md](design/rotation/rotation_readme.md) | 导航索引 | 所有人 | 5分钟 |
| [rotation_standards.md](design/rotation/rotation_standards.md) | 理论基础 | 开发者/审查者 | 30分钟 |
| [rotation_quick_reference.md](design/rotation/rotation_quick_reference.md) | 快速查询 | 所有人 | 5分钟 |

过程记录（问题分析、项目总结等）见 [dev-notes/](dev-notes/)：`CODE_ANALYSIS_ROTATION.md`、`ROTATION_PROJECT_SUMMARY.md`。

---

## 💡 使用建议

### 新开发者上手
1. 读 [design/rotation/rotation_quick_reference.md](design/rotation/rotation_quick_reference.md)（5分钟）
2. 在编码时查阅 [design/rotation/rotation_readme.md](design/rotation/rotation_readme.md)

### 修改旋转相关代码
1. 参考 [design/rotation/rotation_standards.md](design/rotation/rotation_standards.md) 的具体约定
2. 检查 [design/rotation/rotation_quick_reference.md](design/rotation/rotation_quick_reference.md) 的常见错误
3. 过程记录与改进方案见 [dev-notes/CODE_ANALYSIS_ROTATION.md](dev-notes/CODE_ANALYSIS_ROTATION.md)（开发过程稿）

### 深入学习
遵循 [design/rotation/rotation_readme.md](design/rotation/rotation_readme.md) 中的学习路径

---

## 🚀 计划中的文档

以下主题的文档计划在未来添加：

- [ ] `algorithms/` - 算法文档（SfM, BA, 特征匹配等）
- [ ] `architecture/` - 系统架构文档
- [ ] `api/` - API 参考文档
- [ ] `tutorials/` - 新手教程

---

**最后更新**: 2026-02-08  
**位置**: `doc/README.md`
