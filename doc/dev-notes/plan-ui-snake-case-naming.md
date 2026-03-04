# UI 层函数/方法统一为 snake_case 命名计划

## 1. 目标与原则

- **目标**：全项目（含 algorithm、database、ui）函数与方法统一为 **snake_case**（小写下划线），不再使用驼峰（camelCase）。
- **原则**：
  - 公开接口与私有方法一律 `snake_case`。
  - Qt 槽函数、信号名也统一为 `snake_case`（与规范一致，便于全项目一致）。
  - 类名、枚举、枚举值保持现有规范（PascalCase / kPascalCase）。

## 2. 已完成

### 2.1 Camera 模块（algorithm）

以下已改为 snake_case：

| 原命名 | 新命名 |
|--------|--------|
| `ApplyDistortion` | `apply_distortion` |
| `RemoveDistortion` | `remove_distortion` |
| `UndistortPoint` | `undistort_point` |
| `UndistortPoints` | `undistort_points` |
| `toAlgorithmIntrinsics()` | `to_algorithm_intrinsics()` |

- `has_distortion()` 已是 snake_case，未改。
- 调用处：`resection.cpp`、`incremental_sfm_helpers.cpp`、`isat_intrinsics.h`、`isat_incremental_sfm.cpp` 已同步修改。

### 2.2 编码规范文档

- 在 **doc/design/02_coding_style.md** §2.1 中，将「函数/方法：snake_case 或 camelCase（仅 Qt slots）」改为 **仅 snake_case**，并注明 UI 槽与信号也使用 snake_case。

## 3. UI 层命名规则（约定）

| 类型 | 原风格示例 | 目标风格 |
|------|------------|----------|
| 公开方法 | `LoadCamera`, `GetCamera`, `SetGroupName` | `load_camera`, `get_camera`, `set_group_name` |
| 槽函数 | `onFocalLengthPxEdited`, `onProjectChanged` | `on_focal_length_px_edited`, `on_project_changed` |
| 信号 | `cameraModelChanged`, `fieldModified` | `camera_model_changed`, `field_modified` |
| 内部方法 | `InitializeUI`, `ConnectSignals`, `UpdateUIState` | `initialize_ui`, `connect_signals`, `update_ui_state` |
| 布尔/查询 | `validateCamera`, `isLoaded`, `maybeSave` | `validate_camera`, `is_loaded`, `maybe_save` |

- **Qt 重写**：如 `closeEvent`, `showEvent` 等 Qt 虚函数名保持不变（Qt API 约定）。
- **Qt 模型/委托重写**：如 `setData`, `data`, `rowCount` 等 Qt 虚函数保持与 Qt 一致，不重命名。

## 4. UI 文件范围与建议顺序

按目录分批，便于 review 与测试。

### 4.1 工具类 / 无 Qt 信号槽

| 文件 | 说明 |
|------|------|
| `utils/coordinates.h`, `utils/coordinates.cpp` | `parseCoordinates` → `parse_coordinates`，`isOK` → `is_ok`，`isProject` → `is_project` |
| `utils/qstring_convert.h` | 若有函数则改为 snake_case |
| `system_config.h`, `system_config.cpp` | `setConfigPath` → `set_config_path`，`configPath` → `config_path`，`loadCoordinateDatabases` → `load_coordinate_databases`，`isLoaded` → `is_loaded` |

### 4.2 Widgets（按依赖从少到多）

| 文件 | 说明 |
|------|------|
| `widgets/camera_model_widget.h/.cpp` | `setCameraModel`, `getCameraModel`, `validateCamera`, `clearAll`, `cameraModelChanged`, `onParameterChanged`, `initializeUI`, `updateValidationStatus`, `updateCameraModel` → 对应 snake_case；所有 `connect` 同步 |
| `widgets/camera_parameter_editor_widget.h/.cpp` | `LoadCamera`, `GetCamera`, `SetGroupName`, `GetGroupName`, `ShowGroupNameField`, `SetEditable`, `SetMode`, `GetMode`, `fieldModified`, `modeChanged`, `autoEstimateRequested`，以及所有 `on*` 槽、`InitializeUI`, `ConnectSignals`, `UpdateUIByMode`, `UpdateGroupLevelUI`, `UpdateImageLevelUI`, `UpdateRigBasedUI` → snake_case；connect 与调用处同步 |
| `widgets/coordinate_system_config_widget.h/.cpp` | 所有 `on*`、`initializeUI`, `connectSignals`, `updateUIState`, `validateForm`, `validate*Mode` → snake_case |
| `widgets/spatial_reference_dialog.h/.cpp` | `onItemClicked`, `onAddFavorite`, `onFilter`, `onClearFilter`, `onClearFavorite`, `showTrees`, `showAll`, `showSome` → snake_case |
| `widgets/gnss_measurement_import_dialog.h/.cpp` | `setFile`, `setCoordinateType`, `setImportRotation`, `setUseUniformCovariance`, `setUniformCovariance`, `checkFieldData` → snake_case |
| `widgets/gps_points_wizard_dialog.h/.cpp` | `setFile`, `isImportByName`, `isImportRotation`, `updateModel`, `checkEnablePreview`, `validImport`, `enableSelectImportOption`, `getFieldIndex` → snake_case |
| `widgets/gps_points_wizard_model.h/.cpp` | `updateColumnCount`, `setFilter`, `filterInit`, `setDataSource`, `updateDatas` 等 → snake_case |
| `widgets/gps_points_wizard_delegate.h/.cpp` | `setDataBseDocument`, `setEditorData`, `setModelData`, `updateEditorGeometry`（若为自定义非 Qt 虚函数则改；Qt 重写保留）→ snake_case |
| `widgets/image_groups_management_panel.h/.cpp` | `editGroupRequested`, `onNewGroup`, `onImportImages`, `onEditGroup`, `onDeleteGroup`, `onProjectChanged`, `onImageGroupAdded` 等 → snake_case |

### 4.3 Dialogs

| 文件 | 说明 |
|------|------|
| `dialogs/image_group_detail_panel.h/.cpp` | `closeEvent` 保留；`groupDataChanged`, `onCameraParameterModified`, `onCameraParameterModeChanged`, `onGroupNameModified`, `onAutoEstimateRequested` → snake_case |
| `dialogs/coordinate_system_config_dialog.h/.cpp` | `onValidationChanged`, `initializeUI` → snake_case |
| `dialogs/image_editor_dialog.h/.cpp` | `imagesChanged` → `images_changed` |
| `dialogs/image_preview_dialog.h/.cpp` | 所有自定义方法 → snake_case |
| `dialogs/new_at_task_dialog.h/.cpp` | 自定义方法 → snake_case |
| `dialogs/image_group_dialog.h/.cpp` | 自定义方法 → snake_case |
| `dialogs/new_project_dialog.h/.cpp` | 自定义方法 → snake_case |
| `dialogs/project_info_dialog.h/.cpp` | 自定义方法 → snake_case |

### 4.4 Main / Panels / Models

| 文件 | 说明 |
|------|------|
| `main_window.h/.cpp` | `closeEvent`, `showEvent` 保留；所有 `on*`、`createMenuBar`, `createToolBar`, `createWorkspace`, `createStatusBar`, `connectSignalsSlots`, `loadSettings`, `saveSettings`, `maybeSave`, `updateWindowTitle` → snake_case；所有 connect 与调用处同步 |
| `panels/at_task_panel.h/.cpp` | 自定义方法与槽 → snake_case |
| `models/workspace_tree_model.h/.cpp` | `treeRefreshed`, `setProjectDocument`, `refreshTree`，所有 `on*`、`buildTree`, `clearTree`, `updateATTaskNode` 等 → snake_case；Qt 重写 `rowCount`, `columnCount` 等保留 |
| `models/project_document.h/.cpp` | 大量 getter/setter 与业务方法 → 统一 snake_case（如 `getProject`, `setProject` → `get_project`, `set_project`） |

## 5. 实施注意

1. **每次只改一个或少数几个相关文件**，改完即编译、运行与简单 UI 测试，避免大规模一次改导致难以排查。
2. **信号重命名**：信号名改为 snake_case 后，所有 `connect(..., SIGNAL(xxx), ...)` 或 `connect(..., &Class::signal_name, ...)` 需同步修改；若其他文件有 connect 到该信号，需全局搜索该信号名并一起改。
3. **跨文件调用**：重命名后对调用方（如 main_window 调用的 createMenuBar → create_menu_bar）做全文搜索并更新。
4. **文档与注释**：重命名完成后，检查 02_coding_style.md、12_implementation_details.md 等是否还有“Qt slots 可用 camelCase”等表述，统一改为“全项目函数/方法使用 snake_case”。

## 6. 与 02_coding_style 的同步

- 在 **§2.1 总览** 中：
  - 将「函数 / 方法：`snake_case` 或 `camelCase`（仅 Qt slots）」改为：「函数 / 方法：`snake_case`（含 UI 槽与信号）」。
- 可选：在 **§12 CLI 工具规范** 或 **§8 函数设计** 中增加一句：「全项目统一：函数、方法、槽、信号均使用 snake_case，类名与枚举仍为 PascalCase。」

---

**计划状态**：Camera 模块与规范文档更新待执行；UI 层按上述顺序分批执行，每批完成后建议提交并做一次 UI 回归。
