# 08 - Coordinates and rotation conventions

This is the most detail-heavy part of InsightAT: strict definitions for photogrammetric consistency.

## 1. Coordinate systems
Handled through GDAL/PROJ, described by `CoordinateSystemDescriptor`.

### Supported kinds
- **EPSG** — numeric codes (e.g. 4326, 3857)
- **WKT** — OpenGIS WKT strings
- **ENU (East–North–Up)** — local tangent frame anchored at a lon/lat origin
- **Local** — abstract local frame with no geodetic tie

### Vertical datum
Ellipsoidal and orthometric heights can be modeled when a `reference_point` (or equivalent) pins the vertical relationship.

## 2. Rotation conventions

Two families are kept explicit to avoid cross-domain confusion:

### 2.1 Omega–Phi–Kappa (ω, φ, κ)
- **Domain** — classical photogrammetry
- **Definition** — **extrinsic** Z–Y–X
- **Use** — collinearity, standard aerial/AT workflows

### 2.2 Yaw–pitch–roll
- **Domain** — UAS, navigation, robotics
- **Definition** — **intrinsic** Z–Y′–X″
- **Frame** — often NED or ENU

## 3. Internal normalization
- **Angles** — all internal math in **radians**; the UI may show **degrees**
- **Library** — `rotation_utils.h` converts between OPK, ypr, quaternions, and `Eigen::Matrix3d`
- **Config** — `CoordinateSystemDescriptor::rotation_convention` records the active convention so transforms stay consistent and avoid ad hoc assumptions

For deeper rotation notes (Chinese), see [dev-notes/rotation/rotation_readme.md](../../dev-notes/rotation/rotation_readme.md).
