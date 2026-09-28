# CLI I/O conventions (InsightAT)

This document governs all InsightAT CLIs (`isat_*`) for **stdout**, **stderr**, and **exit codes**, so that:

- Scripts and pipes get stable, parseable behavior
- Logs and third-party noise do not corrupt machine-readable output
- Tools that **mutate** projects or files behave consistently

---

## 1. Basics

- **Exit code**
  - `0` — success
  - non-zero — failure (no “success” payload on stdout when failing)

- **stderr**
  - Logs, hints, warnings, errors (`glog`)
  - Human-readable text only — **not** the primary progress channel

- **stdout**
  - **Machine-readable output only** (`ISAT_EVENT` lines)
  - No user hints, no debug spew, no text progress bars

---

## 2. Machine-readable lines: prefix + one JSON object per line

To survive accidental writes to stdout from libraries (or future extra prints), **every machine-readable line must start with a fixed prefix** followed by **one line of JSON** (NDJSON style).

- **Prefix:** `ISAT_EVENT ` (trailing space)
- **Format:**

```
ISAT_EVENT { ...json... }
```

Rules:

- JSON is **one line** (compact `dump()`) for `grep` / `awk` / log pipelines
- Multiple results ⇒ multiple `ISAT_EVENT` lines
- Human text stays on stderr

---

## 3. Suggested JSON fields

Each `ISAT_EVENT` line should include:

- **`type`** — event name (string)
- **`ok`** — success (bool)
- **`data`** — main payload (object / array / scalar)
- **`error`** — message when `ok` is false (optional)

Examples:

```
ISAT_EVENT {"type":"project.create","ok":true,"data":{"project_path":"demo.iat","uuid":"..."}}
ISAT_EVENT {"type":"project.add_group","ok":true,"data":{"group_id":1,"group_name":"DJI"}}
ISAT_EVENT {"type":"project.add_images","ok":true,"data":{"group_id":1,"added":200}}
```

Failure:

```
ISAT_EVENT {"type":"project.add_group","ok":false,"error":"project file not found"}
```

The process also exits non-zero.

---

## 4. Log levels (shared by all `isat_*`)

| Option | Behavior |
|--------|----------|
| `--log-level=LEVEL` | `error`, `warn`, `info`, or `debug` (highest priority) |
| `-v` / `--verbose` | same as `--log-level=info` |
| `-q` / `--quiet` | same as `--log-level=error` |

**Priority (high → low):** `--log-level` > `-q` > `-v` > default `warn`.

| Level | Meaning | Typical use |
|-------|---------|-------------|
| error | errors only | silent scripts |
| warn | warnings and above | default |
| info | informational | follow main steps |
| debug | info + `VLOG(1)` | deep debugging |

Implementation: `error` / `warn` / `info` map to glog `minloglevel`; `debug` also sets `FLAGS_v >= 1` for `VLOG(1)`.

---

## 5. Data Container Format (IDC) Considerations

When modifying the InsightAT Data Container (IDC) readers/writers ([IDCReader](../../../../../src/algorithm/io/idc_reader.h)/[IDCWriter](../../../../../src/algorithm/io/idc_writer.h)), special care must be taken to preserve all original blob descriptor fields:

- **Always preserve original blob fields** - When optimizing [IDCReader](../../../../../src/algorithm/io/idc_reader.h) for performance (e.g., O(1) lookups), ensure the [`get_blob_descriptor`](../../../../../src/algorithm/io/idc_reader.h) method returns all original fields from the JSON descriptor, especially critical ones like `dtype`.
- **Critical fields** - The `dtype`, `shape`, `offset`, and `size` fields are essential for downstream components to properly interpret binary data.
- **Backward compatibility** - Changes should maintain compatibility with existing data files.

See [13_idc_format_spec.md](13_idc_format_spec.md) for the complete specification.

---

## 6. Progress (`ISAT_EVENT`)

Progress is reported on **stdout** as `ISAT_EVENT` lines (not stderr `PROGRESS:`).

### 6.1 `type=progress`

```
ISAT_EVENT {"type":"progress","ok":true,"data":{
  "step":"extract",
  "step_index":2,
  "step_count":6,
  "fraction":0.42,
  "overall":0.28,
  "current":42,
  "total":100,
  "unit":"images",
  "message":"Extracting SIFT"
}}
```

| Field | Meaning |
|-------|---------|
| `step` | Stable id: `create`, `extract`, `match`, `tracks`, `seed_eval`, `incremental_sfm`, `undistort` |
| `step_index` / `step_count` | 1-based index within the active step list for this run |
| `fraction` | Progress **within the current step**, in `[0, 1]` |
| `overall` | Pipeline progress in `[0, 1]`; **driver fills this** when forwarding |
| `current` / `total` / `unit` | Optional fine-grained counters |
| `message` | Short English label for UI |

**Overall formula (driver):** `overall = (step_index - 1 + fraction) / step_count`.

Child tools may emit a minimal progress event (only `fraction` / `current` / `total` / `unit` / `message`); `isat_sfm` enriches with `step`, indices, and `overall` before appending to `events.ndjson`.

### 6.2 Step / pipeline boundaries

```
ISAT_EVENT {"type":"step.start","ok":true,"data":{"step":"match","step_index":3,"step_count":6}}
ISAT_EVENT {"type":"step.end","ok":true,"data":{"step":"match","elapsed_s":12.4}}
ISAT_EVENT {"type":"pipeline.start","ok":true,"data":{"run_id":"...","steps":["extract","match"],"log_dir":"..."}}
ISAT_EVENT {"type":"pipeline.end","ok":true,"data":{"run_id":"...","elapsed_s":123.4}}
```

### 6.3 Pipeline log directory (`isat_sfm`)

Each run writes:

```
<work>/logs/
  current.json                 # {"run_id","dir"}
  run_YYYYMMDD_HHMMSS/
    meta.json
    console.log                # human summary (UI default)
    detail.log                 # full glog / child stderr
    events.ndjson              # ISAT_EVENT only (progress source of truth)
```

UI and scripts should **tail** these files rather than relying on a pipe.

### 6.4 Legacy

`PROGRESS: 0.35` on stderr is **legacy**. Do not use it in new code.

---

## 7. Migration

Legacy tools that emit raw JSON (no prefix) can migrate to `ISAT_EVENT` over time. During migration:

- New / interactive tools should follow this spec strictly
- Large export commands may add `--isat-event` to switch to prefixed one-line JSON
