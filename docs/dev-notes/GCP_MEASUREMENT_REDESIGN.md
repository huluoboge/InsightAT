# GCP Measurement Multi-Image Observation Redesign

**Date**: 2026-02-08  
**Status**: ✅ Implemented and Compiled  
**Version**: database_types v5 with GCPMeasurement v2

---

## 1. Problem Statement

The original GCPMeasurement structure supported only a single image observation per measurement:

```cpp
// ❌ OLD DESIGN: Single observation only
struct GCPMeasurement {
    uint32_t gcp_id;
    double x, y, z;           // 3D coordinate
    double cov_xx, cov_yy, cov_zz;
    double pixel_x, pixel_y;  // Only ONE image observation!
};
```

**Issue**: In typical photogrammetric workflows, a ground control point (GCP) is observed in multiple images simultaneously. Each observation provides a 2D pixel constraint in a different image. The original design had no way to associate pixel measurements with their corresponding images.

**Real-world scenario**:
- GCP #42 is visible in Image 10, 15, and 20
- Each image records a different pixel location for the same physical point
- Need to associate pixels with image IDs

---

## 2. Solution: Multi-Observation Vector

Redesigned GCPMeasurement to contain a vector of observations:

```cpp
// ✅ NEW DESIGN: Multiple observations per GCP
struct GCPMeasurement {
    struct Observation {
        uint32_t image_id;           // Which image observes this GCP
        double pixel_x, pixel_y;     // Pixel coordinates in that image
        double pixel_cov_x, pixel_cov_y;  // Uncertainty in pixels (optional)
    };
    
    uint32_t gcp_id;
    std::string gcp_name;            // Human-readable name (NEW)
    double x, y, z;                  // 3D ground coordinate
    double cov_xx, cov_yy, cov_zz;
    double cov_xy, cov_xz, cov_yz;   // Full covariance matrix (NEW)
    std::vector<Observation> observations;  // All image observations
};
```

### Key Improvements:

1. **Multi-Image Support** ✅
   - One GCPMeasurement = One complete GCP with all observations
   - Each observation explicitly links to an image_id
   - Natural representation of photogrammetric constraints

2. **Pixel Uncertainty** ✅
   - Optional pixel_cov_x and pixel_cov_y per observation
   - Allows weighted constraints in bundle adjustment
   - Accounts for measurement quality per observation

3. **Full Covariance** ✅
   - Added off-diagonal terms: cov_xy, cov_xz, cov_yz
   - Proper representation of 3D coordinate uncertainty
   - More complete error propagation in BA

4. **Human Identification** ✅
   - Added gcp_name field for user reference
   - Avoids relying solely on numeric gcp_id
   - Improves data interpretation

---

## 3. Design Patterns

### Nested Observation Struct

The `Observation` nested struct provides clear organization:

```cpp
struct Observation {
    uint32_t image_id;           // Foreign key to Image
    double pixel_x, pixel_y;     // 2D pixel location
    double pixel_cov_x = 0.0, pixel_cov_y = 0.0;  // Optional uncertainty
    
    // Serialization handled by Cereal
    template<class Archive>
    void serialize(Archive& ar) {
        ar(image_id, pixel_x, pixel_y, pixel_cov_x, pixel_cov_y);
    }
};
```

**Benefits**:
- Encapsulates all observation data
- Clear semantic meaning
- Easy to iterate over observations
- Can be extended with additional per-observation metadata

### Using the New Structure

```cpp
// Create a GCP measurement
Measurement m;
m.type = Measurement::Type::kGCP;

m.gcp.gcp_id = 42;
m.gcp.gcp_name = "GCP_CORNER_A";
m.gcp.x = 1000.5;
m.gcp.y = 2000.5;
m.gcp.z = 100.5;

// Full 3D covariance
m.gcp.cov_xx = 0.01;
m.gcp.cov_yy = 0.01;
m.gcp.cov_zz = 0.01;
m.gcp.cov_xy = 0.002;
m.gcp.cov_xz = 0.001;
m.gcp.cov_yz = 0.001;

// Add observations from Image 10
m.gcp.observations.push_back({
    image_id: 10,
    pixel_x: 123.45,
    pixel_y: 567.89,
    pixel_cov_x: 0.5,
    pixel_cov_y: 0.5
});

// Add observations from Image 15
m.gcp.observations.push_back({
    image_id: 15,
    pixel_x: 234.56,
    pixel_y: 456.78,
    pixel_cov_x: 0.5,
    pixel_cov_y: 0.5
});

// Add observations from Image 20
m.gcp.observations.push_back({
    image_id: 20,
    pixel_x: 345.67,
    pixel_y: 345.67,
    pixel_cov_x: 0.6,
    pixel_cov_y: 0.6
});
```

---

## 4. Validation Logic

### IsValid() Implementation

The updated `GCPMeasurement::IsValid()` method now checks:

```cpp
bool Measurement::GCPMeasurement::IsValid() const {
    // Check GCP ID
    if (gcp_id == static_cast<uint32_t>(-1)) {
        return false;
    }
    
    // Check 3D coordinate validity
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return false;
    }
    
    // IMPORTANT: Must have at least one observation
    if (observations.empty()) {
        return false;
    }
    
    // Check each observation
    for (const auto& obs : observations) {
        if (obs.image_id == static_cast<uint32_t>(-1)) {
            return false;
        }
        if (!std::isfinite(obs.pixel_x) || !std::isfinite(obs.pixel_y)) {
            return false;
        }
    }
    
    return true;
}
```

### Validation Rules:

1. `gcp_id` must not be -1 (invalid sentinel)
2. 3D coordinates (x, y, z) must be finite
3. **observations vector must not be empty** (new requirement)
4. Each observation's image_id must be valid (not -1)
5. Each observation's pixel coordinates must be finite

---

## 5. Backward Compatibility & Version Control

### Cereal Versioning

```cpp
CEREAL_CLASS_VERSION(insight::database::Measurement::GCPMeasurement, 2);  // v2: Multi-observation
CEREAL_CLASS_VERSION(insight::database::Measurement::GCPMeasurement::Observation, 1);
```

### Version History:

| Version | Changes |
|---------|---------|
| v1 | Original: single pixel_x/y fields |
| v2 | **NEW**: observations vector, gcp_name, full covariance |

### Migration Path:

Old files with v1 GCPMeasurement will not auto-load as v2 (breaking change). Consider:

1. **Option A**: Version bump with manual migration
   - Load v1 data
   - Convert single (pixel_x, pixel_y) to observations[0]
   - Infer image_id from context or metadata

2. **Option B**: Accept breaking change
   - Existing data incompatible
   - Benefit: Cleaner design, no legacy code
   - Suitable if data is not critical

Current implementation uses **Option B** (no auto-migration).

---

## 6. Serialization

### Full Serialization Template

```cpp
// In header file
template<class Archive>
void serialize(Archive& ar, uint32_t version) {
    if (version == 2) {
        ar(gcp_id, gcp_name, x, y, z, 
           cov_xx, cov_yy, cov_zz, cov_xy, cov_xz, cov_yz,
           observations);
    }
}
```

The vector serialization is handled automatically by Cereal for all elements.

---

## 7. Photogrammetry Context

### Why This Design Matters

In bundle adjustment (BA), GCPs are constraints that:

1. **Provide absolute scale** - Link local coordinate system to world coordinates
2. **Improve orientation** - Multiple observations per GCP reduce drift
3. **Constrain multiple images** - A single GCP constrains multiple camera poses

Traditional BA software (Bundler, VisualSfM, Colmap, CERES) all represent GCPs this way:

```
GCP #42
  └─ Observation in Image 10: (pixel_x=123.45, pixel_y=567.89)
  └─ Observation in Image 15: (pixel_x=234.56, pixel_y=456.78)
  └─ Observation in Image 20: (pixel_x=345.67, pixel_y=345.67)
```

### Bundle Adjustment Implications

When solving with multiple observations per GCP:

```
Residuals for GCP #42:
  r_10 = (proj(R_10 * X + t_10) - (123.45, 567.89))² / pixel_cov_10
  r_15 = (proj(R_15 * X + t_15) - (234.56, 456.78))² / pixel_cov_15
  r_20 = (proj(R_20 * X + t_20) - (345.67, 345.67))² / pixel_cov_20

Total = w_gcp * (r_10 + r_15 + r_20)
```

Our design directly supports this:

```cpp
// In BA solver:
for (auto& measurement : gcp_measurements) {
    for (auto& obs : measurement.gcp.observations) {
        double reprojection_error = calculate_error(obs.image_id, obs.pixel_x/y);
        // Apply optional per-observation weighting
        residual += obs.pixel_cov_x * reprojection_error;
    }
}
```

---

## 8. Compilation Results

```
✅ Compilation: SUCCESS
  - 0 errors
  - 0 database_types.cpp warnings (1 unrelated Qt5 deprecation in GUI)
  - All targets built successfully

Modified Files:
  - src/database/database_types.h
    * GCPMeasurement struct (lines 177-222)
    * Added Observation nested struct
    * Updated serialization template
    * Bumped Cereal version to v2
  
  - src/database/database_types.cpp
    * GCPMeasurement::IsValid() implementation (lines 136-161)
    * Enhanced validation with observations vector check
```

---

## 9. Summary

| Aspect | Before | After |
|--------|--------|-------|
| **Observations** | Single (pixel_x, pixel_y) | Vector of Observation structs |
| **Image Association** | Implicit/Unclear | Explicit (image_id per obs) |
| **Pixel Uncertainty** | No | Optional (pixel_cov_x/y) |
| **3D Covariance** | Diagonal only (3 terms) | Full matrix (6 terms) |
| **GCP Identification** | Numeric ID only | Numeric ID + name string |
| **Design Pattern** | Flat structure | Nested Observation struct |
| **Validation** | 2 checks | 5 checks (including observations) |
| **Photogrammetry Alignment** | Partial | ✅ Full alignment |

---

## 10. Future Enhancements

Potential extensions (not implemented):

1. **Observation Flags** - Mark outliers, disabled observations
2. **Temporal Data** - Observation timestamp
3. **Quality Metrics** - RMS residual per observation
4. **Reference Confidence** - Trust in GCP ground truth
5. **Metadata** - Collection method, reference standards used

These can be added to the `Observation` struct without breaking the vector design.

---

## References

- **Bundle Adjustment**: Triggs et al. (1999), "Bundle Adjustment — A Modern Synthesis"
- **Photogrammetry**: Luhmann et al. (2014), "Close-range photogrammetry and 3D imaging"
- **Cereal Serialization**: https://uscilab.github.io/cereal/
- **InsightAT Design**: See CAMERA_MODEL_DESIGN.md, PROJECT_STRUCTURE_DESIGN.md, IMAGEGROUP_DESIGN.md

