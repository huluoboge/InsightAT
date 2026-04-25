# 开发笔记（`doc/dev-notes/`）

**语言：本目录全文使用中文。** 其余 `doc/` 区域（`user/`、`develop/` 等）以英文为准，见 [`../README.md`](../README.md)。

本目录是维护者自己的**开发过程记录**：排障、迁移、与 Agent 协作的草稿、阶段性计划等。**未定稿、高度依赖当时上下文**的内容优先放这里；**相对稳定的规范与系统设计**在 [`../develop/design/`](../develop/design/index.md)。

## 性质

- **过程稿，非对外唯一事实源**：可能与当前代码不一致，**以仓库代码为准**。
- **个人/心路向**：适合追溯「当时为什么这样改」，不等同于用户手册。
- **可增删**：过时内容可归档或删除，不视为对外承诺。

## 目录里常见内容

| 区域 | 说明 |
|------|------|
| 根下大量 `*.md` | 实现报告、REFACTOR 记录、日期笔记、设计草案 |
| `rotation/` | 旋转表示与实现相关的**过程**笔记（与 `develop/design/08_coordinate_and_rotation.md` 等对照阅读） |
| `retrieval/` | 检索相关过程稿 |
| `tools/` | 各 `isat_*` 工具在开发中的设计/测试记录（过程向，非 [`../user/`](../user/README.md) 里的使用手册） |
| `third_party/` | 第三方库集成、迁移记录 |
| `RELEASE_PLAN_v0.1.md` | 发布计划过程稿（若存在） |

## 和 `doc/` 其他目录的关系

| 目录 | 用途 |
|------|------|
| [`../user/`](../user/README.md) | **使用与操作**入口（与根 README / Docker 对齐） |
| [`../develop/`](../develop/README.md) | **规范与正式设计**（`develop/design/` 编号文档） |
| **`dev-notes/`（本目录）** | **个人开发心路**与未定稿 |
| [`../experiment/`](../experiment/) | 实验与随笔 |

正式设计入口： **[develop/design/index.md](../develop/design/index.md)**。
