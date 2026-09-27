# 06 - UI framework and patterns

The InsightAT UI is built on **Qt 5.15**, with a strict split between core logic and presentation.

## 1. Document / view
- **`ProjectDocument` (`src/ui/models/ProjectDocument.h`)** — Central controller. Owns the live `Project` and all mutations (e.g. add images, set CRS). Emits `projectChanged()` after edits.
- **`MainWindow`** — Top-level shell. Connects to `ProjectDocument` via signals/slots and updates menus and title from project state.

## 2. Model / view
- **`WorkspaceTreeModel`** — `QAbstractItemModel` over the `Project` tree: Project → image groups → images, AT tasks → subtasks. Backs a `QTreeView`.

## 3. Composed widgets
Business logic is pushed into reusable widgets:
- **`CoordinateSystemWidget`** — Search, WKT input, common EPSG shortcuts.
- **`CameraModelWidget`** — Intrinsics and distortion, with validation (e.g. positive focal length).
- **`ProjectInfoDialog`** — Project metadata editor.

## 4. Signal/slot workflow
1. User confirms a dialog.
2. Dialog emits `dataCreated(Data)`.
3. `MainWindow` calls `ProjectDocument::addData`.
4. `ProjectDocument` updates `Project` and emits `projectUpdated()`.
5. Tree and property views refresh.

## 5. UI rules
- **Modality** — Use modal dialogs for critical state (e.g. CRS) to avoid races.
- **Async work** — Long jobs (BA, bulk import) off the UI thread, with `QFutureWatcher` or custom signals for completion.
