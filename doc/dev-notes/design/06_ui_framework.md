# 06 - UI 框架与设计模式

InsightAT 的 UI 基于 **Qt 5.15** 构建，遵循核心逻辑与界面展示分离的原则。

## 1. Document/View 模式
- **ProjectDocument (`src/ui/models/ProjectDocument.h`)**: UI 的核心控制器。它持有真实的 `Project` 对象，并负责所有对项目的修改操作（如 AddImage, SetCRS）。
    - 修改数据后，它会发射 `projectChanged()` 信号。
- **MainWindow**: 全局窗口容器。它通过信号槽连接到 `ProjectDocument`，并根据项目状态更新菜单和标题栏。

## 2. Model/View 架构
- **WorkspaceTreeModel**: 这是一个 `QAbstractItemModel` 的实现。
    - 它直接映射了 `Project` 的层级结构。
    - 项目树节点包括：Project -> Image Groups -> Images, AT Tasks -> Sub Tasks。
    - 它为 `QTreeView` 提供数据展示。

## 3. 面向组件的 Widget 设计
复杂业务逻辑被封装在独立的小部件中，提高复用性：
- **CoordinateSystemWidget**: 包含搜索、WKT 输入和常用 EPSG 列表。
- **CameraModelWidget**: 负责复杂的相机内参和畸变参数计算/展示，包含实时验证逻辑（如焦距必须大于 0）。
- **ProjectInfoDialog**: 专门用于编辑项目的基本元数据。

## 4. 信号槽驱动的工作流
UI 的交互遵循典型的异步/响应式模式：
1. 用户在对话框点击“完成”。
2. 对话框发射 `dataCreated(Data data)` 信号。
3. `MainWindow` 接收信号，调用 `ProjectDocument::addData(data)`。
4. `ProjectDocument` 修改底层 `Project` 实例，发射 `projectUpdated()`。
5. 所有关联的 View (Tree, Property Panel) 自动刷新。

## 5. UI 规范
- **模态性**：对于关键配置（如 CRS），使用模态对话框防止状态冲突。
- **异步处理**：耗时操作（如 BA 计算、海量图像导入）应在非 UI 线程运行，并通过 `QFutureWatcher` 或自定义信号通知界面。
