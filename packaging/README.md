# Packaging (Ubuntu 22.04 + CUDA 12.8)

Supported platforms: **Ubuntu 22.04** and **Windows** with **CUDA 12.8**.  
Default product is **CLI-only** (`isat_*`). Qt is optional (`packaging/legacy/`).

## Developer (local, friendly)

## Developer (local)

If `~/.local/ceres-cuda128` exists, use it; otherwise apt `libceres-dev`.

**cuDSS:** CMake (and the build script) pick `libcudss/<CUDA_major>/` from the
detected toolkit (CUDA 12 → cuDSS 12). Do not rely on the unversioned
`.../cmake/cudss` package — it often points at the newest tree (e.g. 13).

```bash
./packaging/linux/build.sh
# binaries: ./build/isat_*

INSIGHTAT_USE_SYSTEM_CERES=1 ./packaging/linux/build.sh   # force apt Ceres
```

CI/Docker release images still build their own Ceres + cuDSS.

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
