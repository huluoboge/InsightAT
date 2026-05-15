# 13 - IDC Format Specification (InsightAT Data Container)

The InsightAT Data Container (IDC) is a binary format designed for efficient storage and retrieval of feature data, descriptors, and related metadata. This specification defines the format and blob conventions to ensure compatibility across all components.

## 1. Overall Format

The IDC binary format consists of:

- **Magic Header:** `"ISAT"` (4 bytes)
- **Format Version:** `uint32_t` (4 bytes) - Current version: 1
- **JSON Size:** `uint64_t` (8 bytes) - Size of JSON descriptor in bytes
- **JSON Descriptor:** UTF-8 string (variable length) - Contains metadata about blobs
- **Padding:** 0-7 bytes to align next section to 8-byte boundary
- **Binary Payload:** Raw data blobs (8-byte aligned)

## 2. JSON Descriptor Schema

The JSON descriptor contains metadata about each blob in the container:

```json
{
  "schema_version": "1.0",
  "task_type": "feature_extraction",
  "blobs": [
    {
      "name": "keypoints",
      "dtype": "float32",
      "shape": [N, 4],
      "offset": 0,
      "size": 12800
    },
    {
      "name": "descriptors", 
      "dtype": "uint8",
      "shape": [N, 128],
      "offset": 12800,
      "size": 32768
    }
  ],
  "algorithm": {...},
  "metadata": {...}
}
```

### 2.1 Blob Descriptor Fields

Each blob descriptor in the `blobs` array must contain:

- **`name`** - Unique identifier for the blob (string)
- **`dtype`** - Data type ("float32", "uint8", "int32", etc.) - **CRITICAL FIELD**
- **`shape`** - Array dimensions [rows, cols] or [size] (array of integers)
- **`offset`** - Byte offset from start of payload (integer)
- **`size`** - Size of blob data in bytes (integer)

**Important:** The `dtype` field is critical for proper data interpretation and must never be omitted.

## 3. Blob Access Pattern

Components access blobs using the [IDCReader](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L30-L68) class:

```cpp
IDCReader reader("file.isat_feat");
auto blob_desc = reader.get_blob_descriptor("descriptors");  // Returns full descriptor with all fields
auto data = reader.read_blob<uint8_t>("descriptors");
```

## 4. Reader Implementation Notes

The [IDCReader](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L30-L68) maintains an O(1) lookup index for blob names but must preserve all original fields from the JSON descriptor when returning blob information. The [get_blob_descriptor](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L53-L53) method must return the complete original descriptor including `dtype`, `shape`, and other metadata.

## 5. Common Blob Types

### 5.1 Feature Extraction Blobs
- **`keypoints`** - `dtype: "float32"`, `shape: [N, 4]` (x, y, scale, orientation)
- **`descriptors`** - `dtype: "uint8"` or `dtype: "float32"`, `shape: [N, D]` (N features, D dimensions)

### 5.2 Matching Blobs
- **`matches`** - `dtype: "uint32"`, `shape: [M, 2]` (index pairs)
- **`match_scores`** - `dtype: "float32"`, `shape: [M]` (confidence scores)

## 6. Compatibility Requirements

When modifying the [IDCReader](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_reader.h#L30-L68) or [IDCWriter](file:///home/jones/Git/01jones/InsightAT/src/algorithm/io/idc_writer.h#L77-L97):

1. **Preserve all original fields** when returning blob descriptors
2. **Never remove critical fields** like `dtype`, especially when optimizing lookup performance
3. **Maintain backward compatibility** with existing files
4. **Update all dependent components** when changing format conventions

## 7. Common Pitfalls

- **Missing `dtype` field**: Results in crashes when downstream components try to interpret data
- **Incomplete blob descriptors**: Components expecting full metadata fail silently or crash
- **Optimization without verification**: Performance improvements should not break format expectations

## 8. Verification

After any changes to IDC handling:

1. Verify all blob fields are preserved in read operations
2. Test with existing data files
3. Confirm downstream components receive expected metadata
4. Run end-to-end pipeline tests