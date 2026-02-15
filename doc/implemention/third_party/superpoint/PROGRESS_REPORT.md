# SuperPoint Integration - Progress Report

**Date**: February 16, 2026  
**Status**: Phase 1 Core Infrastructure Complete ✅  
**Next**: isat_extract Runtime Integration

---

## Completed Work ✅

### 1. IDC Metadata Schema v1.1 (Step 1) ✅

**Files Modified**:
- `src/algorithm/io/idc_writer.h`
- `src/algorithm/io/idc_reader.h`
- `src/algorithm/io/idc_reader.cpp`
- `src/algorithm/tools/isat_extract.cpp`

**Changes**:
- Added `DescriptorSchema` struct with fields:
  - `feature_type` (sift/superpoint)
  - `descriptor_dim` (128/256)
  - `descriptor_dtype` (uint8/float32)
  - `normalization` (l2/none)
  - `quantization_scale` (512.0 for SIFT uint8, 1.0 for float)
- Extended `createFeatureMetadata()` to accept optional `DescriptorSchema`
- Implemented `getDescriptorSchema()` with v1.0 fallback logic for backward compatibility
- Updated `isat_extract` to populate schema when writing SIFT features

**Verification**:
- ✅ Compiles without errors
- ✅ Backward compatible with old IDC files (v1.0)
- ✅ New IDC files write schema_version=1.1

---

### 2. SuperPoint Extractor Module (Step 3) ✅

**Files Created**:
- `src/algorithm/modules/extraction/superpoint_extractor.h`
- `src/algorithm/modules/extraction/superpoint_extractor.cpp`

**Implementation**:
- **SuperPointConfig**: Model path, provider (CPU/CUDA), threshold, NMS radius, max keypoints
- **SuperPointExtractor**: PIMPL pattern with ONNX Runtime C++ API
  - Input: cv::Mat (grayscale or BGR)
  - Output: cv::KeyPoint (x, y, scale=1.0, angle=0), descriptors (Nx256, float32, L2-normalized), scores
  - NMS: Local maximum suppression with configurable radius
  - Thread-safe: Each instance owns its ONNX session (suitable for StageCurrent)
- **Graceful Degradation**: Stub implementation when `!INSIGHTAT_HAS_SUPERPOINT` (compile error with actionable message)

**Features**:
- Automatic grayscale conversion
- L2 normalization of descriptors in-place
- Top-k keypoint selection by confidence score
- CUDA provider detection via `isCudaAvailable()`

---

### 3. CMake Optional Dependency (Step 5) ✅

**Files Modified**:
- `CMakeLists.txt` (top-level)
- `src/algorithm/CMakeLists.txt`

**Configuration** (as modified by user):
- Added `INSIGHTAT_ENABLE_SUPERPOINT` option (default: ON)
- ONNX Runtime path: `/opt/onnxruntime-gpu`
- CUDA Toolkit: `/usr/local/cuda-11.8`
- Sets `SUPERPOINT_AVAILABLE` flag when ONNXRuntime found
- Defines `INSIGHTAT_HAS_SUPERPOINT` for conditional compilation
- Links `onnxruntime`, `cudart`, `cudnn` to `InsightATAlgorithm`
- Graceful fallback: Builds without SuperPoint if ONNX Runtime missing

**Verification**:
- ✅ CMake configuration succeeds with ONNX Runtime
- ✅ Build succeeds: `make InsightATAlgorithm` completes without errors
- ✅ Configuration messages:
  ```
  -- ONNXRuntime found - SuperPoint backend enabled
  -- InsightATAlgorithm: SuperPoint support enabled
  --   Linking ONNX Runtime: onnxruntime
  --   ONNX Include: /opt/onnxruntime-gpu/include
  ```

---

### 4. CLI Parameters & Backend Selection (Step 2+4 Partial) ✅

**File Modified**:
- `src/algorithm/tools/isat_extract.cpp`

**CLI Changes**:
- Added `--feature-type` option (sift|superpoint, default: sift)
- Added SuperPoint-specific parameters:
  - `--superpoint-model` (path to .onnx file, required for SuperPoint)
  - `--superpoint-provider` (cpu|cuda, default: cpu)
  - `--superpoint-threshold` (default: 0.005)
  - `--superpoint-nms-radius` (default: 4)
  - `--superpoint-max-keypoints` (default: 1024)
- Updated help text to tag SIFT vs SuperPoint parameters
- Added validation:
  - Feature type must be sift or superpoint
  - SuperPoint model file must exist
  - Compile-time check for `INSIGHTAT_HAS_SUPERPOINT`
  - Actionable error messages for missing dependencies

**ImageTask Extension**:
- Added SuperPoint fields:
  - `sp_keypoints` (std::vector<cv::KeyPoint>)
  - `sp_descriptors` (cv::Mat, Nx256 CV_32F)
  - `sp_scores` (std::vector<float>)
  - Retrieval variants: `sp_keypoints_retrieval`, etc.

**Logging**:
- Backend-specific configuration logging (SIFT vs SuperPoint parameters)
- Feature type prominently displayed in startup logs

**Verification**:
- ✅ `./isat_extract --help` shows all parameters with correct documentation
- ✅ Compiles without errors
- ✅ Parameter validation logic in place

---

## Work In Progress 🚧

### 5. isat_extract Runtime Integration (Step 2+4 Remaining)

**Current State**:
- CLI parameters: ✅ Complete
- ImageTask data structures: ✅ Extended
- Extraction stage branching: 🚧 Started (added `if (feature_type == "sift")` wrapper)
- SuperPoint extraction lambda: ❌ Not implemented yet
- IDC write_features adaptation: ❌ Needs SuperPoint branch

**Remaining Tasks**:
1. **Add SuperPoint extraction stage** (paralleling SIFT logic):
   ```cpp
   } else if (feature_type == "superpoint") {
   #ifdef INSIGHTAT_HAS_SUPERPOINT
       insight::modules::SuperPointConfig sp_config;
       sp_config.model_path = superpoint_model_path;
       sp_config.provider = superpoint_provider;
       sp_config.threshold = superpoint_threshold;
       sp_config.nms_radius = superpoint_nms_radius;
       sp_config.max_keypoints = superpoint_max_keypoints;
       
       insight::modules::SuperPointExtractor sp_extractor(sp_config);
       
       StageCurrent spStage("SuperPoint", 1, GPU_QUEUE_SIZE,
           [&image_tasks, &sp_extractor, ...](...) {
               // Extract from task.image → task.sp_keypoints, task.sp_descriptors
               // Extract from task.image_retrieval (if needed)
           });
   #else
       LOG(FATAL) << "SuperPoint not compiled in";
   #endif
   }
   ```

2. **Adapt postProcessStage** for SuperPoint (no normalization/uint8 needed, descriptors already L2-normalized)

3. **Adapt writeStage** to handle SuperPoint features:
   ```cpp
   // Write SuperPoint features
   if (!task.sp_keypoints.empty()) {
       // Convert cv::KeyPoint → keypoint blob (x, y, score, 0)
       // Write descriptors (Nx256 float32)
       // Populate DescriptorSchema

(feature_type="superpoint", dim=256, dtype="float32")
   }
   ```

4. **Update pipeline chaining** to use correct stage variable (siftGPUStage vs spStage)

**Estimated Effort**: 2-3 hours (mostly adapting existing SIFT logic)

---

## Pending Work ⏳

### 6. Retrieval Pipeline Dimension De-Hardcoding (Step 6)

**Scope**:
- `isat_retrieve.cpp`: Remove descriptor_dim==128 check, read from IDC
- `isat_train_vlad.cpp`: Accept variable descriptor_dim from features
- `isat_train_vocab.cpp`: Same as above
- `vlad_encoding.cpp`: Replace hardcoded 128 with runtime dimension
- `vlad_retrieval.cpp`: Validate query/database dimension match
- `vocab_tree_retrieval.cpp`: Same as VLAD
- `pca_whitening.cpp` (if applicable): Dimension-agnostic whitening

**Risk**: High dimension mismatch potential if not validated strictly

---

### 7. Runtime Protection & Error Observability (Step 7)

**Tasks**:
- Schema validation at module boundaries (extract/train/retrieve)
- Fast-fail on dimension mismatch with actionable errors
- Enhanced logging: feature_type, descriptor_dim, provider in all CLI tools
- Progress bar integration: Show backend in description

---

### 8. Documentation & Design Sync (Step 8)

**Files to Update**:
- `doc/design/08_functional_at_toolkit.md`: Add SuperPoint section
- `doc/tools/isat_extract.md`: SuperPoint usage examples
- `third_party/superpoint/README.md`: InsightAT integration guide
- `doc/implemention/third_party/superpoint/SUPERPOINT_INTEGRATION_PLAN.md`: Mark steps complete

---

## Known Limitations & Future Work

**Phase 1 (Current)**:
- ❌ SuperPoint dual-output mode not yet implemented (matching + retrieval features)
- ❌ isat_match does not support SuperPoint (will error on 256-dim features)
- ❌ UI (ATTaskPanel) has no SuperPoint selection option

**Phase 2 (Future)**:
- Matching pipeline integration (requires BFMatcher or FLANN for 256-dim)
- UI integration (feature_type dropdown in extraction dialog)
- SuperPoint model management (download/version/packaging)
- Performance optimization (batched inference for retrieval features)

---

## Build & Test Status

**Last Build**: February 16, 2026
- ✅ `make InsightATAlgorithm` - Success
- ✅ `make isat_extract` - Success
- ✅ CMake configuration with ONNX Runtime - Success
- ⏳ Runtime extraction test - Pending (awaits completion of Step 4)

**Next Milestone**: Complete isat_extract integration → run end-to-end test with real images

---

## How to Continue

### Option A: Complete Step 4 (isat_extract Runtime)
1. Implement SuperPoint extraction stage (see "Remaining Tasks" above)
2. Adapt write_features lambda for SuperPoint
3. Test with sample images: `./isat_extract --feature-type=superpoint --superpoint-model=/path/to/model.onnx -i images.json -o /tmp/features`
4. Verify IDC files contain correct schema (dim=256, dtype=float32)

### Option B: Proceed to Step 6 (Retrieval De-Hardcoding)
- Can be done in parallel if Step 4 is delegated
- Lower risk, purely refactoring existing code
- Enables SuperPoint features to flow through retrieval pipeline

### Option C: Add Basic Integration Test
- Create simple test: extract → train_vlad → retrieve
- Use small dataset (10-20 images)
- Verify 256-dim codebook creation and retrieval works

---

## Code Quality & Conventions

**Adherence**:
- ✅ Namespace: `insight::modules`, `insight::io`
- ✅ PIMPL pattern for ONNX dependency isolation
- ✅ Graceful degradation (#ifdef INSIGHTAT_HAS_SUPERPOINT)
- ✅ cmdLine parameter style with .doc()
- ✅ Glog for logging
- ✅ No Qt in algorithm layer
- ✅ Backward-compatible IDC schema

---

**Report End**
