# 归档快照 · 2026-09-28

> **这是一份冻结的历史快照，不描述当前系统。**

| 项目 | 内容 |
|------|------|
| 快照日期 | 2026-09-28 |
| 代码基线 | `main` 分支，提交 `305bbd1` |
| 用途 | 历史追溯：为什么这样设计、哪些方案被放弃、旧文档当时写了什么 |
| 内容 | 旧 `docs/` 除 `index.html` 与 `images/` 之外的全部文档 |

**当前文档只有两处：**

- `docs/index.html` —— 项目主页（GitHub Pages 源为 `/docs`）。
- [`docs/TECHNICAL_STATUS.md`](../TECHNICAL_STATUS.md) —— 中文技术现状，按当前实现描述。
- [`docs/TECHNICAL_STATUS_EN.md`](../TECHNICAL_STATUS_EN.md) —— English technical status.

## 快照内容

| 目录 | 原文用途 |
|------|----------|
| `develop/` | 工程规范与设计稿（英文），含 `develop/design/` 编号设计集 |
| `dev-notes/` | 开发过程笔记（中文），含逐工具记录与实验草稿 |
| `experiment/` | 实验与草稿（中文） |
| `report/` | 早期技术报告（中文），是当前报告的主要素材 |
| `user/` | 面向用户的使用与集成说明 |
| `archive/` | 更早一轮已归入遗留的设计稿（01/06/08/10）及原因表 |
| `DOCS_MAP.md` | 快照当时的 `docs/README.md`，描述旧目录分层 |

## 使用须知

1. **不要把这些内容当作当前能力引用。** 其中多份文档写的是构想中的形态，与 `305bbd1` 的实际实现不符（例如把 Qt UI 写成产品界面、把 CRS/任务层写成求解能力、把已实现的 SiftGPU CUDA 12.x 支持写成未适配）。
2. 若旧文档与代码冲突，**以代码为准**。
3. 快照内约 65 个相对链接在归档时即已失效（指向从未存在过的路径，如 `src/Common/rotation_utils.h`、`docs/dev-notes/rotation/*`）；这是归档前的历史欠账，未做修补。
4. 未实现能力的汇总见快照内 `develop/design/14_roadmap.md`；本快照之后的实现进展以当前报告与 git 历史为准。
