# Packaging (Ubuntu 22.04 + CUDA 12.8)

Supported platforms: **Ubuntu 22.04** and **Windows** with **CUDA 12.8**.  
Default product is **CLI-only** (`isat_*`). Qt is optional (`packaging/legacy/`).

## Developer (local, friendly)

Uses system Ceres (`apt install libceres-dev`) — **forces** apt Ceres so a leftover
`~/.local/ceres-cuda128` does not get picked up. No cuDSS required for local builds.

```bash
# Linux
./packaging/linux/build.sh
# binaries: ./build/isat_*

# Optional (advanced): your own CUDA Ceres + cuDSS
INSIGHTAT_USE_CUDA_CERES=1 ./packaging/linux/build.sh
```

CI/Docker release images still build custom Ceres + cuDSS; local script does not change that.

Windows: open the repo with vcpkg toolchain + `vcpkg.json` (ordinary `ceres`), configure with `-DINSIGHTAT_BUILD_QT_UI=OFF`.

## Release packages (CI / Docker)

Builds custom Ceres (hy) + cuDSS inside the image, then ships AppImage + `.deb`:

```bash
./packaging/docker-build.sh run
# → ./build-appimage/*.AppImage
# → ./build-deb/*.deb
```

Windows zip is produced by CI via `packaging/windows/package.ps1`.

## Layout

| Path | Role |
|------|------|
| `linux/build.sh` | Local cmake build |
| `Dockerfile` + `docker-build.sh` | Release image |
| `appimage/build.sh` | AppImage |
| `deb/package.sh` | Debian package |
| `windows/package.ps1` | Windows zip staging |
| `legacy/qt-gui.sh` | Optional Qt GUI |
