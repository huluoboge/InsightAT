# InsightAT 文档目录

本目录包含 InsightAT 项目的文档，按用途分类。

## 📁 目录结构

```
doc/
├── README.md                    ← 你在这里
├── RELEASE_PLAN_v0.1.md         ← 版本发布计划（过程稿）
├── experiment/                  ← 实验与讨论草稿
├── dev-notes/                   ← 开发笔记 + 历史设计稿
│   ├── design/                  ← 原 doc/design/ 设计文档（过程/历史稿，以代码为准）
│   └── third_party/             ← 第三方集成过程记录
└── tools/                       ← 工具使用说明、快速上手、测试记录
```

顶层 **`doc/design/` 已移除**：原编号设计文档与旋转子文档已迁入 **`doc/dev-notes/design/`**，便于与实现过程笔记放在同一棵树下维护；**与当前实现不一致时以仓库代码为准**。

## 🎯 快速导航

### 设计类文档（dev-notes/design）

架构、数据模型、CLI 约定、旋转表示法等**历史设计稿**入口：

- [dev-notes/design/index.md](dev-notes/design/index.md)

### 开发笔记（dev-notes）

实现报告、修复总结、迁移计划等**过程稿**：

- [dev-notes/README.md](dev-notes/README.md)

### 构建与上手

- [dev-notes/build.md](dev-notes/build.md) — 依赖、`cmake`、Docker 等（从根 README 拆分时可对照维护）

### 工具说明（tools）

- 见 [tools/](tools/) 目录内各 `*.md`

---

## 旋转表示法（Rotation）

**入口**: [dev-notes/design/rotation/rotation_readme.md](dev-notes/design/rotation/rotation_readme.md)

| 文件 | 用途 |
|------|------|
| [rotation_readme.md](dev-notes/design/rotation/rotation_readme.md) | 导航索引 |
| [rotation_standards.md](dev-notes/design/rotation/rotation_standards.md) | 理论基础与约定 |
| [rotation_quick_reference.md](dev-notes/design/rotation/rotation_quick_reference.md) | 快速查询 |

过程性分析见 [dev-notes/CODE_ANALYSIS_ROTATION.md](dev-notes/CODE_ANALYSIS_ROTATION.md)、[dev-notes/ROTATION_PROJECT_SUMMARY.md](dev-notes/ROTATION_PROJECT_SUMMARY.md)。

---

## 📚 文档贡献指南

1. **确定分类** — 过程稿放 `dev-notes/`；工具说明放 `tools/`。
2. **更新导航** — 在 `dev-notes/README.md` 或本文件补充链接。
3. **链接代码** — 注释中请使用 `doc/dev-notes/design/...` 路径，避免引用已删除的 `doc/design/`。

### 代码注释示例

```cpp
/**
 * 相关文档：doc/dev-notes/design/rotation/rotation_standards.md
 */
struct DBPose {
    // ...
};
```

---

## 💡 使用建议

### 新开发者

1. 读 [dev-notes/design/rotation/rotation_quick_reference.md](dev-notes/design/rotation/rotation_quick_reference.md)（约 5 分钟）
2. 需要时再打开 [dev-notes/design/index.md](dev-notes/design/index.md) 按主题查阅

### 修改旋转相关代码

1. [rotation_standards.md](dev-notes/design/rotation/rotation_standards.md)
2. [rotation_quick_reference.md](dev-notes/design/rotation/rotation_quick_reference.md)
3. 过程记录：[dev-notes/CODE_ANALYSIS_ROTATION.md](dev-notes/CODE_ANALYSIS_ROTATION.md)

---

**最后更新**: 2026-04-24  
**位置**: `doc/README.md`
