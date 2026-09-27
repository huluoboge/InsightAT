# 设计：SfM 日志落盘、Tail 与 ISAT_EVENT 进度

- **状态**：已实现
- **日期**：2026-09-27
- **范围**：`isat_sfm` 落盘布局、子工具 `progress` 事件、Node `sfm-gui` tail + 进度条

## 目标

1. 落盘是日志真相源；UI 用类似 `tail -f` 读文件，不再以 pipe 刷屏为主。
2. 人读日志与机器事件分文件。
3. 进度一律 `ISAT_EVENT` `type=progress`（及 `step.*` / `pipeline.*`）。

## 目录

```
work/logs/
  current.json
  run_YYYYMMDD_HHMMSS/
    meta.json
    console.log
    detail.log
    events.ndjson
```

## 进度事件

见 [05_cli_io_conventions.md](../develop/design/05_cli_io_conventions.md) §6。

`overall = (step_index - 1 + fraction) / step_count`（由 `isat_sfm` 补齐写入 `events.ndjson`）。

## GUI

- 订阅 `logs/current.json` → 轮询 `console.log` / `detail.log` / `events.ndjson`
- Console | Detail 切换；进度条读最新 `progress` / `pipeline.end`
