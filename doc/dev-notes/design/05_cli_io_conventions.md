# CLI I/O 规范（InsightAT）

本规范约束所有 InsightAT CLI 工具（`isat_*`）在 **stdout / stderr / exit code** 上的行为，目的是：

- 让 CLI 易于被脚本/管道调用（稳定、可解析）
- 避免日志/第三方输出污染机器可读结果
- 统一各工具的交互方式（尤其是“会修改工程/输出文件”的命令）

---

## 1. 基本原则

- **退出码（exit code）**
  - `0`：成功
  - `!=0`：失败（失败时不输出成功结果）

- **stderr**
  - 仅用于：日志、提示、警告、错误、进度（例如 `PROGRESS: ...`）
  - `glog` 默认输出到 stderr，符合本规范

- **stdout**
  - 仅用于：机器可读输出
  - 不允许输出人类提示、调试信息、进度条

---

## 2. 统一的机器可读输出格式：前缀 + 单行 JSON

为抵抗潜在的第三方库误写 stdout（或未来扩展导致的额外输出），**所有机器可读输出必须以固定前缀开头**，后接 **单行 JSON**（NDJSON 风格），一条消息一行。

- **前缀**：`ISAT_EVENT `
- **格式**：

```
ISAT_EVENT { ...json... }
```

约束：

- JSON 必须是 **单行**（compact `dump()`），便于 `grep`/`awk`/日志系统处理
- 若一个命令需要输出多条结果，也必须是多行 `ISAT_EVENT ...`
- 人类可读信息必须去 stderr

---

## 3. 建议的 JSON schema（事件消息）

每条 `ISAT_EVENT` 的 JSON 建议包含：

- **`type`**：事件类型（字符串）
- **`ok`**：是否成功（布尔）
- **`data`**：主要数据（对象/数组/数值）
- **`error`**：失败时的错误信息（字符串，可选）

示例：

```
ISAT_EVENT {"type":"project.create","ok":true,"data":{"project_path":"demo.iat","uuid":"..."}}
ISAT_EVENT {"type":"project.add_group","ok":true,"data":{"group_id":1,"group_name":"DJI"}}
ISAT_EVENT {"type":"project.add_images","ok":true,"data":{"group_id":1,"added":200}}
```

失败示例：

```
ISAT_EVENT {"type":"project.add_group","ok":false,"error":"project file not found"}
```

同时进程返回非 0。

---

## 4. 日志级别（统一约定）

所有 `isat_*` CLI 工具统一支持以下日志级别选项，便于脚本与人工调试时控制输出量。

- **选项**
  - `--log-level=LEVEL`：显式指定级别（优先级最高）。`LEVEL` 取值：`error`、`warn`、`info`、`debug`。
  - `-v` / `--verbose`：等价于 `--log-level=info`。
  - `-q` / `--quiet`：等价于 `--log-level=error`。

- **优先级**（从高到低）：`--log-level` > `-q` > `-v` > 默认 `warn`。

- **级别语义**（与 glog 对应）

| 级别   | 含义           | 典型用途         |
|--------|----------------|------------------|
| error  | 仅错误         | 静默脚本、仅看失败 |
| warn   | 警告及以上     | 默认，平衡可读与噪音 |
| info   | 信息及以上     | 查看主要步骤     |
| debug  | 信息 + VLOG(1) | 开发调试、排查问题 |

实现上：`error`/`warn`/`info` 对应 glog 的 `minloglevel`；`debug` 在 info 基础上设置 `FLAGS_v>=1` 以输出 `VLOG(1)` 等详细日志。

---

## 5. 进度输出（可选）

若命令需要进度：

- 进度输出必须写入 **stderr**
- 推荐格式（示例）：

```
PROGRESS: 0.35
```

其中数值范围 `[0, 1]`。

---

## 6. 兼容性说明

历史命令如果已经输出纯 JSON（无前缀），后续可逐步迁移到 `ISAT_EVENT`。迁移期内建议：

- 新增/交互类命令严格执行本规范
- 导出型命令（如输出大 JSON）可提供 `--isat-event` 选项切换到带前缀单行输出

