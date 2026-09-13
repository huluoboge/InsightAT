# 开发笔记（`doc/dev-notes/`）

**语言：本目录全文使用中文。** 其余 `doc/` 区域（`user/`、`develop/` 等）以英文为准，见 [`../README.md`](../README.md)。

本目录是维护者的**开发过程记录**：排障、迁移、与 Agent 协作的草稿、阶段性计划和实验记录。内容可能依赖当时的代码状态，**以当前代码和正式设计文档为准**。

## 当前活动内容

根目录只保留尚未归类的临时记录。已经按语义整理的内容进入以下分类：

- [`design/`](design/README.md)：设计与架构；
- [`imp/`](imp/README.md)：实现、迁移、进度和测试结果；
- [`req/`](req/README.md)：需求、计划和待办；
- [`reference/`](reference/README.md)：使用指南、操作参考和可视化资料。

第三方集成过程仍在 [`third_party/`](third_party/)；旧的工具目录只保留尚未完成迁移的工具文档。

已经完成的阶段总结、历史日报和旧发布计划见 [`archive/`](archive/README.md)。正式规范和发布说明统一放在 [`../develop/`](../develop/README.md)。

## 文档性质

- **过程稿，非对外唯一事实源**：可能与当前代码不一致，以仓库代码为准。
- **用于追溯上下文**：适合了解某次改动的背景，不等同于用户手册或稳定 API 规范。
- **允许归档**：完成或失去时效的记录进入 `archive/`，不视为对外承诺。

## 和 `doc/` 其他目录的关系

| 目录 | 用途 |
|------|------|
| [`../user/`](../user/README.md) | **使用与操作**入口（与根 README / Docker 对齐） |
| [`../develop/`](../develop/README.md) | **规范、正式设计与发布说明** |
| **`dev-notes/`（本目录）** | **开发过程、未定稿设计与专题笔记** |
| [`../experiment/`](../experiment/) | 实验与随笔 |
| [`archive/`](archive/README.md) | 已完成报告、历史日志和旧计划 |

正式设计入口：**[develop/design/index.md](../develop/design/index.md)**。
