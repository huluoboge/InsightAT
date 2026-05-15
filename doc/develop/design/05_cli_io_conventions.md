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
  - Logs, hints, warnings, errors, progress (e.g. `PROGRESS: …`)
  - `glog` goes to stderr by default — aligned with this policy

- **stdout**
  - **Machine-readable output only**
  - No user hints, no debug spew, no progress bars

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

When modifying the InsightAT Data Container (IDC) readers/writers ([IDCReader](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L30-L68)/[IDCWriter](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_writer.h#L77-L97)), special care must be taken to preserve all original blob descriptor fields:

- **Always preserve original blob fields** - When optimizing [IDCReader](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L30-L68) for performance (e.g., O(1) lookups), ensure the [get_blob_descriptor](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L53-L53) method returns all original fields from the JSON descriptor, especially critical ones like `dtype`.
- **Critical fields** - The `dtype`, `shape`, `offset`, and `size` fields are essential for downstream components to properly interpret binary data.
- **Backward compatibility** - Changes should maintain compatibility with existing data files.

See [13_idc_format_spec.md](13_idc_format_spec.md) for the complete specification.

---

## 6. Progress (optional)

If a tool reports progress:

- Write to **stderr**
- Suggested form:

```
PROGRESS: 0.35
```

Value in `[0, 1]`.

---

## 7. Migration

Legacy tools that emit raw JSON (no prefix) can migrate to `ISAT_EVENT` over time. During migration:

- New / interactive tools should follow this spec strictly
- Large export commands may add `--isat-event` to switch to prefixed one-line JSON
