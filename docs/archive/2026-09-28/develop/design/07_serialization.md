# 07 - Data persistence (serialization)

InsightAT uses two persistence mechanisms, for two different jobs.

| What | Format | Written by | Notes |
|------|--------|-----------|-------|
| Project file (`project.iat`) | **Cereal JSON archive** | `isat_project`, `isat_camera_estimator` | Human-readable, diffable, and easy to inspect when a run misbehaves. Wrapped as `make_nvp("project", project)`. |
| Pipeline artifacts (`.isat_feat`, `.isat_match`, `.isat_geo`, `.isat_tracks`) | **IDC binary container** | stage CLIs | Self-describing JSON header + 8-byte-aligned binary payload. See [13_idc_format_spec.md](13_idc_format_spec.md). |
| Inter-stage metadata (`images_all.json`, `pairs_*.json`, `poses.json`, `sfm_timing.json`, `camera_estimate_meta.json`) | **Plain JSON** (nlohmann/json) | stage CLIs | Deliberately simple; the pair JSONs are the pipeline's inter-stage contract. |
| Data-model round-trips in tests | Cereal **binary** archives | `test_serialization*`, `test_project_serialization` | Exists and is exercised; the CLI path uses JSON. |

Cereal stays the baseline abstraction because it supports both JSON and binary archives over the same `serialize()` functions and carries explicit versioning.

## 1. Why Cereal?

- **Low overhead** — compact streams, fast load
- **Header-only** — no heavy codegen step
- **STL-friendly** — `std::vector`, `std::optional`, `std::map`, `std::string`, …
- **Versioning built in** — `CEREAL_CLASS_VERSION` + a versioned `serialize()`

## 2. Versioning

Core types carry explicit schema versions so older files keep loading.

```cpp
// 1. Declare a version
CEREAL_CLASS_VERSION(MyType, 1);

// 2. serialize with a version
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    if (version == 0) {
        ar(CEREAL_NVP(old_field));
    } else {
        ar(CEREAL_NVP(old_field));
        ar(CEREAL_NVP(new_rotation_field));
        ar(CEREAL_NVP(is_valid));
    }
}
```

This is used in practice, not just in principle — e.g. `CEREAL_CLASS_VERSION(insight::database::ATTask::InputSnapshot, 2)` guards the `image_groups` field that was added after version 0.

## 3. Rules and notes

- **Never remove a field from `serialize()`** without keeping a backward path for the old version; add fields behind a version check instead.
- **`CEREAL_NVP`** — always name fields so JSON archives stay readable and a future format switch stays possible.
- **Load failures** usually surface as `cereal::Exception` (or a parse error from the JSON archive). Callers must catch them and report version skew rather than crashing.
- **Binary portability** — binary archives are not guaranteed across compilers/ABIs; the supported targets are 64-bit Linux and Windows.
- **IDC is not Cereal.** Binary pipeline payloads go through the IDC reader/writer, which has its own compatibility rules (preserve every blob descriptor field, especially `dtype`).

## 4. See also

- [13_idc_format_spec.md](13_idc_format_spec.md) — the binary container used for pipeline artifacts
- [09_data_model.md](09_data_model.md) — the types being serialized
- [11_architecture_overview.md](11_architecture_overview.md) — where each artifact appears in a run
