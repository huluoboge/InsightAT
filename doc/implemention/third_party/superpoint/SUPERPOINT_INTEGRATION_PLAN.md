# SuperPoint Integration Plan for InsightAT

**Date**: February 16, 2026  
**Status**: In Progress  
**Scope**: Extract + Retrieve (Matching deferred to Phase 2)

## Executive Summary

This document outlines the plan to integrate SuperPoint ONNX C++ implementation (located in `third_party/superpoint`) as an alternative feature extraction backend alongside SIFT in InsightAT. The integration will enable SuperPoint to participate in the full extraction and retrieval pipeline while maintaining compatibility with existing SIFT workflows.

### Key Constraints & Decisions

- **CLI-First**: Each tool invocation runs exactly one feature type (SIFT or SuperPoint), never both simultaneously
- **Dual-Track Support**: Project can use both 128-dim (SIFT) and 256-dim (SuperPoint) features across different runs
- **ONNX Runtime**: First release supports both CPU and CUDA Execution Providers
- **IDC Schema**: Upgrade to v1.1 with backward-compatible new fields
- **Phase 1 Scope**: `isat_extract` + `isat_retrieve` (+ training tools); `isat_match` deferred to Phase 2
- **Thread Model**: Preserve existing StageCurrent semantics for GPU inference
- **Minimal Intrusion**: Keep SIFT path unchanged; add SuperPoint as parallel code path

---

## Architecture Overview

### Current State (SIFT-Only)
```
isat_extract (SIFT) → IDC (128-dim, dtype=uint8/float32)
                       ↓
                  isat_retrieve (VLAD/Vocab) ← codebook (128-dim)
```

### Target State (SIFT + SuperPoint)
```
isat_extract --feature_type=sift → IDC (128-dim, schema v1.1)
                                    ↓
                               isat_retrieve ← codebook (128-dim)

isat_extract --feature_type=superpoint → IDC (256-dim, schema v1.1)
    (ONNX Runtime CPU/CUDA)              ↓
                                    isat_retrieve ← codebook (256-dim)
```

### Key Design Principles

1. **Single Backend per Run**: CLI tools enforce exclusive feature_type selection
2. **Metadata-Driven Workflow**: Descriptor dimension/dtype read from IDC metadata, not hardcoded
3. **Graceful Degradation**: Building without ONNXRuntime only disables SuperPoint backend
4. **Backward Compatibility**: New IDC reader can parse old (v1.0) files via fallback logic

---

## Implementation Steps

### Step 1: Define Feature Specification & IDC Metadata Contract

**Files to Modify**:
- `src/algorithm/io/idc_writer.h`
- `src/algorithm/io/idc_writer.cpp`
- `src/algorithm/io/idc_reader.h`
- `src/algorithm/io/idc_reader.cpp`

**Tasks**:
1. Add `DescriptorSchema` struct:
   ```cpp
   struct DescriptorSchema {
       std::string feature_type;        // "sift", "superpoint"
       int descriptor_dim;               // 128, 256
       std::string descriptor_dtype;     // "uint8", "float32"
       std::string normalization;        // "l2", "none"
       float quantization_scale;         // 512.0 for SIFT uint8, 1.0 for float
       std::string schema_version;       // "1.1"
   };
   ```

2. Extend IDC metadata JSON with optional `descriptor_schema` field
3. IDC writer: populate schema from extractor parameters
4. IDC reader: 
   - Primary: parse `descriptor_schema` if present
   - Fallback: derive from `blobs.descriptors.shape[1]` + `algorithm.parameters.feature_type`

**Acceptance Criteria**:
- Old IDC files (v1.0, no `descriptor_schema` field) readable with correct dimension inference
- New IDC files (v1.1) carry full schema and pass validation

---

### Step 2: Abstract Extraction Backend Entry (No Changes to SIFT Logic)

**Files to Modify**:
- `src/algorithm/tools/isat_extract.cpp`

**Tasks**:
1. Add `--feature_type` CLI parameter (default: `sift`, options: `sift|superpoint`)
2. Add feature-specific parameter groups:
   - SIFT: existing parameters (unchanged)
   - SuperPoint: `--superpoint_model_path`, `--superpoint_provider` (cpu/cuda), `--superpoint_threshold`, `--superpoint_max_keypoints`
3. Early validation: exit with error if SuperPoint selected but not compiled in
4. Branch extraction pipeline based on `feature_type`:
   ```cpp
   if (feature_type == "sift") {
       // Existing SIFT path (unchanged)
   } else if (feature_type == "superpoint") {
       // New SuperPoint path (Step 3)
   }
   ```

**Acceptance Criteria**:
- `--help` shows all parameters grouped by feature type
- Invalid `--feature_type` exits with clear error message
- No SIFT behavior changes

---

### Step 3: Integrate SuperPoint Inference Module

**New Files to Create**:
- `src/algorithm/modules/extraction/superpoint_extractor.h`
- `src/algorithm/modules/extraction/superpoint_extractor.cpp`

**Reference Source**:
- `third_party/superpoint/src/superpoint_inference.cpp` (refactor into library)

**Tasks**:
1. Extract `SuperPointONNX` class into reusable library (under `insight::algorithm::extraction` namespace)
2. API alignment:
   ```cpp
   struct SuperPointConfig {
       std::string model_path;
       std::string provider;  // "cpu", "cuda"
       float threshold;
       int max_keypoints;
   };
   
   class SuperPointExtractor {
   public:
       explicit SuperPointExtractor(const SuperPointConfig& config);
       bool Extract(const cv::Mat& image, 
                    std::vector<cv::KeyPoint>& keypoints,
                    cv::Mat& descriptors,
                    std::vector<float>& scores);
   private:
       // ONNX Runtime session, input/output tensors
   };
   ```

3. Output format: keypoints (x, y, scale=1.0, angle=0), descriptors (Nx256 float32), scores (Nx1)
4. Thread safety: Each extractor owns its ONNX session; safe for StageCurrent usage
5. Error handling: Model loading failures, inference errors with actionable messages

**Acceptance Criteria**:
- Standalone test can run inference on single image
- Descriptors match third_party demo output (within tolerance)
- Memory/GPU context properly released on extractor destruction

---

### Step 4: CLI Parameters & Runtime Configuration Alignment

**Files to Modify**:
- `src/algorithm/tools/isat_extract.cpp` (extend from Step 2)

**Tasks**:
1. Parameter registration (cmdLine style):
   ```cpp
   cmd.add(make_option("superpoint-model", superpoint_model_path, "superpoint_model")
       .doc("Path to SuperPoint ONNX model file"));
   cmd.add(make_option("superpoint-provider", superpoint_provider, "superpoint_provider")
       .doc("ONNX Runtime provider: cpu or cuda (default: cpu)"));
   ```

2. Validation:
   - If `--feature_type=superpoint`, require `--superpoint_model_path`
   - Check model file exists and readable
   - Validate provider against compiled backends

3. Logging:
   - Log feature_type, model path, provider, descriptor_dim at startup
   - Progress bar shows feature type in description

**Acceptance Criteria**:
- `isat_extract --feature_type=superpoint` without model path exits with clear error
- Help text shows SuperPoint parameters with descriptions
- Logs clearly indicate active backend

---

### Step 5: CMake Optional Dependency Integration

**Files to Modify**:
- `CMakeLists.txt` (top-level)
- `src/algorithm/CMakeLists.txt`
- `third_party/CMakeLists.txt`
- `third_party/superpoint/CMakeLists.txt` (refactor from executable to library)

**Tasks**:
1. Add CMake option:
   ```cmake
   option(INSIGHTAT_ENABLE_SUPERPOINT "Enable SuperPoint feature extraction" ON)
   ```

2. Find ONNXRuntime (quiet mode):
   ```cmake
   if(INSIGHTAT_ENABLE_SUPERPOINT)
       find_package(onnxruntime QUIET)
       if(onnxruntime_FOUND)
           set(SUPERPOINT_AVAILABLE TRUE)
       else()
           message(STATUS "ONNXRuntime not found, SuperPoint backend disabled")
           set(SUPERPOINT_AVAILABLE FALSE)
       endif()
   endif()
   ```

3. Refactor `third_party/superpoint`:
   - Change from `add_executable(superpoint_inference)` to `add_library(superpoint STATIC)`
   - Export headers: `superpoint_extractor.h`
   - Link dependencies: onnxruntime, OpenCV, CUDA (conditional)

4. Link to `InsightATAlgorithm`:
   ```cmake
   if(SUPERPOINT_AVAILABLE)
       target_link_libraries(InsightATAlgorithm PUBLIC superpoint)
       target_compile_definitions(InsightATAlgorithm PUBLIC INSIGHTAT_HAS_SUPERPOINT)
   endif()
   ```

5. Runtime checks in code:
   ```cpp
   #ifdef INSIGHTAT_HAS_SUPERPOINT
       // SuperPoint code
   #else
       throw std::runtime_error("SuperPoint not compiled in");
   #endif
   ```

**Acceptance Criteria**:
- Build with ONNXRuntime: all targets succeed
- Build without ONNXRuntime: tools build, SuperPoint selection exits with "not compiled in"
- No dependency pollution to SIFT-only builds

---

### Step 6: Retrieval Pipeline Dimension De-Hardcoding

**Files to Modify**:
- `src/algorithm/tools/isat_retrieve.cpp`
- `src/algorithm/tools/isat_train_vlad.cpp`
- `src/algorithm/tools/isat_train_vocab.cpp`
- `src/algorithm/modules/retrieval/vlad_encoding.h`
- `src/algorithm/modules/retrieval/vlad_encoding.cpp`
- `src/algorithm/modules/retrieval/vlad_retrieval.cpp`
- `src/algorithm/modules/retrieval/vocab_tree_retrieval.cpp`

**Tasks**:

1. **Remove hardcoded `descriptor_dim = 128`**:
   - Replace with runtime dimension read from IDC metadata or codebook header
   - Validate dimension consistency across features and codebook

2. **Unify `uint8` ↔ `float32` conversion semantics**:
   - Current issue: VLAD uses `/512`, vocab uses direct cast
   - Solution: Follow `quantization_scale` from descriptor_schema
   - Default: `512.0` for SIFT backward compatibility, `1.0` for SuperPoint

3. **Update codebook formats**:
   - VLAD codebook header: add `descriptor_dim` field (was implicit 128)
   - Vocab tree: read dimension from training metadata
   - Both: validate training and query dimensions match

4. **API changes**:
   ```cpp
   // Before
   VLADEncoding::Encode(const std::vector<cv::Mat>& descriptors, ...);
   
   // After
   VLADEncoding::Encode(const std::vector<cv::Mat>& descriptors, 
                        const DescriptorSchema& schema, ...);
   ```

**Acceptance Criteria**:
- Train VLAD codebook on SuperPoint 256-dim features → codebook stores dim=256
- Retrieve with 256-dim query against 256-dim database → correct results
- Retrieve with 128-dim query against 256-dim database → early error with dimension mismatch
- Old 128-dim codebooks still work with SIFT features

---

### Step 7: Runtime Protection & Error Observability

**Files to Modify**:
- All CLI tools (`isat_extract`, `isat_retrieve`, `isat_train_*`)

**Tasks**:

1. **Schema validation at boundaries**:
   - Extract: validate extractor output matches declared schema
   - Retrieve: validate query features match database/codebook dimension
   - Train: validate all input features have consistent schema

2. **Explicit logging** (using glog):
   ```cpp
   LOG(INFO) << "Feature backend: " << feature_type 
             << ", descriptor_dim: " << descriptor_dim
             << ", dtype: " << descriptor_dtype
             << ", provider: " << provider;
   ```

3. **Fast-fail error messages**:
   ```
   ERROR: Dimension mismatch: query features are 256-dim but codebook expects 128-dim
   ERROR: SuperPoint model not found: /path/to/model.onnx
   ERROR: CUDA provider requested but ONNXRuntime built with CPU-only
   ```

4. **Progress bar integration**:
   - Show feature type in description: `[SuperPoint CUDA] Extracting features: 45%`

**Acceptance Criteria**:
- Dimension mismatch caught before expensive computation
- Logs contain enough info for offline debugging
- Error messages actionable (what's wrong + how to fix)

---

### Step 8: Documentation & Design Conventions Sync

**Files to Modify/Create**:
- `doc/design/08_functional_at_toolkit.md` (add SuperPoint section)
- `doc/tools/isat_extract.md` (if exists, or create)
- `third_party/superpoint/README.md` (update with InsightAT integration)
- This document (`doc/implemention/third_party/superpoint/SUPERPOINT_INTEGRATION_PLAN.md`)

**Tasks**:

1. **Toolkit documentation**:
   - Add SuperPoint to feature extraction backends table
   - Document CLI parameters with examples
   - Note performance characteristics vs SIFT
   - Clarify Phase 1 vs Phase 2 scope (no matching yet)

2. **SuperPoint README**:
   - Building as part of InsightAT (CMake flags)
   - Model file requirements (where to download, expected format)
   - Provider selection guide (CPU vs CUDA, performance/memory tradeoffs)

3. **Migration guide**:
   - How to run same dataset with SIFT vs SuperPoint
   - Codebook training workflow for each feature type
   - Expected performance differences

**Acceptance Criteria**:
- New users can follow docs to build and run SuperPoint extraction
- Existing SIFT users know how to optionally try SuperPoint
- Phase 2 scope clearly marked as future work

---

## Verification Strategy

### Build Matrix
- **Case 1**: Default build (no ONNXRuntime)
  - Expected: Success, SIFT available, SuperPoint unavailable
- **Case 2**: Build with ONNXRuntime CPU
  - Expected: Success, both backends available, SuperPoint runs on CPU
- **Case 3**: Build with ONNXRuntime CUDA
  - Expected: Success, both backends available, SuperPoint can use CUDA provider

### Functional Tests

| Test Case | Command | Expected Outcome |
|-----------|---------|------------------|
| SIFT extraction (unchanged) | `isat_extract --feature_type=sift ...` | IDC with 128-dim, schema v1.1, same keypoints as before upgrade |
| SuperPoint extraction (CPU) | `isat_extract --feature_type=superpoint --superpoint_model_path=/path/to/model.onnx --superpoint_provider=cpu ...` | IDC with 256-dim, schema v1.1, valid keypoints |
| SuperPoint extraction (CUDA) | Same as above with `--superpoint_provider=cuda` | Faster than CPU, same results |
| Read old IDC (v1.0) | `isat_retrieve` on pre-upgrade IDC | Success, dimension inferred as 128 |
| VLAD training (256-dim) | `isat_train_vlad` on SuperPoint features | Codebook with header dim=256 |
| VLAD retrieval (256-dim) | `isat_retrieve` with 256-dim query + codebook | Valid scores, no crashes |
| Dimension mismatch | `isat_retrieve` with 128-dim query + 256-dim codebook | Error exit with clear message |
| Missing model file | `isat_extract --feature_type=superpoint --superpoint_model_path=/nonexistent` | Error exit with file not found |

### Performance Baselines
- Measure on standard test set (e.g., 1000 images, 1920x1080):
  - SIFT extraction time (baseline, should be unchanged)
  - SuperPoint CPU time (expected: slower than SIFT GPU)
  - SuperPoint CUDA time (expected: competitive with SIFT GPU)
- Memory footprint: ONNX model loading + inference
- Disk usage: IDC file size (256-dim vs 128-dim, ~2x expected)

### Compatibility Checks
- **Backward**: New binaries read old (v1.0) IDC files
- **Forward**: Old binaries cannot read v1.1 IDC (expected graceful failure or ignored fields)
- **Cross-backend**: Extract with SIFT, train VLAD, retrieve → works
- **Cross-backend**: Extract with SuperPoint, train VLAD, retrieve → works
- **Mixed projects**: Project with some images SIFT, some SuperPoint → each tool run uses one type

---

## Risk Assessment & Mitigation

| Risk | Severity | Mitigation |
|------|----------|------------|
| ONNXRuntime ABI incompatibility | High | Document tested versions; provide optional static link |
| CUDA context conflicts (ONNX + SiftGPU) | Medium | Run in separate CLI invocations; investigate context isolation if needed |
| 256-dim breaking existing pipelines | High | Strong validation at boundaries; update all dimension assumptions |
| uint8/float32 quantization semantic drift | Medium | Explicit `quantization_scale` field; unit tests for conversion paths |
| Model file deployment/packaging | Medium | CLI parameter (no default path); document download links |
| Performance regression in SIFT path | High | Comprehensive benchmarking; zero changes to SIFT code sections |

---

## Phase 2 Preview (Out of Scope for Current Plan)

**Deferred to future work**:
- `isat_match` integration: Replace or augment SiftMatchGPU
  - Candidates: BFMatcher (OpenCV), FLANN, HNSW
  - Challenge: 128-dim optimizations in SiftMatchGPU won't transfer
- UI integration: ATTaskPanel parameter exposure for feature_type selection
- Multi-backend projects: UI workflow for managing mixed-feature-type image groups
- Advanced SuperPoint tuning: NMS, descriptor head variations, fine-tuning

---

## Timeline Estimate

| Step | Estimated Effort | Dependencies |
|------|------------------|--------------|
| Step 1 (IDC metadata) | 0.5 day | None |
| Step 2 (Backend abstraction) | 0.5 day | Step 1 |
| Step 3 (SuperPoint module) | 1 day | None (parallel) |
| Step 4 (CLI params) | 0.5 day | Step 2, 3 |
| Step 5 (CMake) | 1 day | Step 3 |
| Step 6 (Retrieval de-hardcode) | 1 day | Step 1 |
| Step 7 (Error handling) | 0.5 day | All above |
| Step 8 (Documentation) | 0.5 day | All above |
| **Testing & Integration** | 1 day | All above |
| **Total** | ~6-7 days | - |

---

## References

- Copilot Instructions: `.github/copilot-instructions.md`
- Architecture Overview: `doc/design/02_architecture_overview.md`
- Implementation Details: `doc/design/07_implementation_details.md`
- Functional Toolkit: `doc/design/08_functional_at_toolkit.md`
- Task Queue Design: `third_party/task_queue/STAGECURRENT_USAGE.md`
- Existing SIFT Integration: `doc/implemention/third_party/siftgpu/`

---

**Document Status**: Draft v1.0  
**Next Review**: After Step 1 completion  
**Change Log**:
- 2026-02-16: Initial draft based on调研 findings and user decisions
