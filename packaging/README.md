# Packaging (Ubuntu 22.04 / 24.04 + CUDA 12.8)

Supported platforms: **Ubuntu 22.04**, **Ubuntu 24.04**, and **Windows** with **CUDA 12.8**.  
Default product is **CLI-only** (`isat_*`). Qt is optional (`packaging/legacy/`).

Release packages are **per-distro**: a Jammy `.deb` will not install on Noble (and vice versa). Prefer the matching AppImage / `.deb`, or the Windows zip.

**Asset naming:** see [RELEASE_ASSETS.md](RELEASE_ASSETS.md) for the unified
`InsightAT-<component>-…` scheme and “which file should I download?” guide.

**`.deb` / AppImage CUDA/cuDSS bundling (22.04 & 24.04):** both
`packaging/deb/package.sh` and `packaging/appimage/build.sh` ship CUDA 12
runtime libs plus a CUDA-12-matched `libcudss` via
`packaging/common/cuda_bundle.sh`. This avoids host `update-alternatives`
pointing `libcudss.so.0` at a CUDA 13 tree that needs `libcublas.so.13`.
Set `CUDA_LIBS_DIR` / `CUDSS_LIB_DIR` (or `CUDSS_ROOT`) when packaging
outside Docker. Packaging fails if linkage still needs `libcublas.so.13`.

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
# Ubuntu 22.04 (default)
./packaging/docker-build.sh run
# → ./build-appimage/*.AppImage
# → ./build-deb/*.deb   (…ubuntu22.04…)

# Ubuntu 24.04
UBUNTU_VERSION=24.04 ./packaging/docker-build.sh run
# → …ubuntu24.04… packages
```

GitHub Actions:

| Workflow | Distro | Release upload |
|---|---|---|
| `linux-build-ubuntu22.yml` | Ubuntu 22.04 | on `release: published` or dispatch with `tag` |
| `linux-build-ubuntu24.yml` | Ubuntu 24.04 | same |
| `windows-build.yml` | Windows | same |
| `electron-gui.yml` | Linux + Windows Electron | same |

CI runs the packaging steps for validation only and does not store AppImage, `.deb`, or zip binaries as Actions artifacts. Release and pre-release assets are uploaded to the GitHub Release instead.

**Draft releases do not trigger workflows.** Publish the release (pre-release checkbox is fine), or run each workflow with `workflow_dispatch` and set `tag` (e.g. `v0.2.5`) to attach assets to an existing draft/pre-release.

Windows zip is produced by CI via `packaging/windows/package.ps1`.

## Layout

| Path | Role |
|------|------|
| `linux/build.sh` | Local cmake build |
| `Dockerfile` | Ubuntu 22.04 release image |
| `Dockerfile.ubuntu24.04` | Ubuntu 24.04 release image |
| `docker-build.sh` | Build/extract helper (`UBUNTU_VERSION=22.04\|24.04`) |
| `common/cuda_bundle.sh` | Shared CUDA 12 + cuDSS bundling / linkage checks |
| `RELEASE_ASSETS.md` | GitHub Release filename scheme + download guide |
| `appimage/build.sh` | AppImage |
| `deb/package.sh` | Debian package (CLI) |
| `deb/package-all.sh` | Meta-package `insightat-all` (Depends: cli + gui + viewer) |
| `windows/package.ps1` | Windows zip staging |
| `legacy/qt-gui.sh` | Optional Qt GUI |
