# nanoflann

**Version**: 1.5.5 (2024-01-15)  
**Source**: https://github.com/jlblancoc/nanoflann  
**License**: BSD-3-Clause  
**Author**: Jose Luis Blanco-Claraco

## Description

nanoflann is a **header-only** C++11 library for building KD-Trees of datasets with different topologies. 
It is a drop-in replacement for `FLANN::KDTreeIndex` with better performance and fewer dependencies.

## Files Included

- `nanoflann.hpp` - Core header-only library (only file needed)

## Usage in InsightAT

Used for spatial indexing in GPS-based image retrieval:
- Build k-d tree from camera positions (GNSS data)
- Radius search for nearby images
- Efficient O(log N) nearest neighbor queries

## Integration

```cpp
#include "nanoflann/nanoflann.hpp"

// Example: 3D point cloud
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    3  // dimensionality
> KDTree;
```

## Commit Hash

Downloaded from master branch: https://raw.githubusercontent.com/jlblancoc/nanoflann/master/include/nanoflann.hpp
Date: 2026-02-13
