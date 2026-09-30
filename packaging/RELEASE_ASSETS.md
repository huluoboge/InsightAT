# Release asset naming

All GitHub Release binaries follow one pattern (from **v0.2.6** onward):

```text
InsightAT-<component>-<version>[-cuda<X.Y>]-<os>-<arch>[-<distro>].<ext>
```

| Field | Meaning | Values |
|-------|---------|--------|
| `component` | What you are downloading | `cli` · `sfm-gui` · `sfm-viewer` · `all` |
| `version` | Package version (**no** leading `v`) | e.g. `0.2.5` |
| `cuda…` | CUDA toolkit the **CLI** was built against | `cuda12.8` (CLI only) |
| `os` | Operating system | `linux` · `windows` |
| `arch` | CPU arch | `x86_64` (AppImage) · `amd64` (`.deb`) · `x64` (Windows zip) · `all` (meta `.deb`) |
| `distro` | Ubuntu ABI the **CLI** links against | `ubuntu22.04` · `ubuntu24.04` (CLI Linux only) |

Checksums sit beside each file as `<same-name>.sha256`.

## Which file do I want?

| Goal | File |
|------|------|
| **Everything (recommended)** | Install CLI + GUI + Viewer debs together, then `InsightAT-all-…-linux-all.deb` (meta package) |
| Command-line `isat_*` on Ubuntu **22.04** | `InsightAT-cli-…-linux-…-ubuntu22.04.AppImage` or `.deb` |
| Same on Ubuntu **24.04** | `…-ubuntu24.04…` |
| Command-line `isat_*` on Windows | `InsightAT-cli-…-windows-x64.zip` |
| Desktop SfM GUI (Electron) | `InsightAT-sfm-gui-…` — **embeds** a reconstruction viewer |
| Standalone reconstruction viewer | `InsightAT-sfm-viewer-…` (also pulled in by `all`) |

### Viewer is a separate executable

`View Reconstruction` launches **`insightat-sfm-viewer`**, not another copy of the GUI.

| Package | Install prefix |
|---------|----------------|
| CLI (`insightat`) | `/usr/lib/insightat` (+ `/usr/bin/isat_*`) |
| GUI (`insightat-sfm-gui`) | `/opt/insightat` |
| Viewer (`insightat-sfm-viewer`) | `/opt/insightat-viewer` |

The GUI `.deb` depends on the viewer package. `insightat-all` pulls CLI + GUI + viewer.

### Install all `.deb`s from a GitHub Release

```bash
# Pick the CLI deb matching your Ubuntu series, then:
sudo dpkg -i \
  InsightAT-cli-*-linux-amd64-ubuntu22.04.deb \
  InsightAT-sfm-gui-*-linux-amd64.deb \
  InsightAT-sfm-viewer-*-linux-amd64.deb \
  InsightAT-all-*-linux-all.deb
sudo apt-get install -f   # if anything is missing
```

The GUI will auto-detect CLI under `/usr/lib/insightat/bin` and `/usr/bin`.

Notes:

- **AppImage** vs **`.deb`**: AppImage is portable (chmod +x and run). `.deb` installs system-wide via `apt`/`dpkg` and must match the Ubuntu series.
- **CLI** packages need an NVIDIA driver compatible with **CUDA 12.8**. GUI/viewer Electron builds do not embed the GPU CLI stack unless you stage `isat_*` into them at build time.
- Do **not** mix `ubuntu22.04` and `ubuntu24.04` packages across distros.

## Examples (`v0.2.5` → after rename)

```text
InsightAT-cli-0.2.5-cuda12.8-linux-x86_64-ubuntu22.04.AppImage
InsightAT-cli-0.2.5-cuda12.8-linux-amd64-ubuntu22.04.deb
InsightAT-cli-0.2.5-cuda12.8-linux-x86_64-ubuntu24.04.AppImage
InsightAT-cli-0.2.5-cuda12.8-linux-amd64-ubuntu24.04.deb
InsightAT-cli-0.2.5-cuda12.8-windows-x64.zip
InsightAT-sfm-gui-0.2.5-linux-x86_64.AppImage
InsightAT-sfm-gui-0.2.5-linux-amd64.deb
InsightAT-sfm-gui-0.2.5-windows-x64.zip
InsightAT-sfm-viewer-0.2.5-linux-x86_64.AppImage
InsightAT-sfm-viewer-0.2.5-linux-amd64.deb
InsightAT-sfm-viewer-0.2.5-windows-x64.zip
InsightAT-all-0.2.5-linux-all.deb
```

## Release title / tag

- **Tag:** `v0.2.6` (leading `v`)
- **Release name:** same as the tag (`v0.2.6`), not `Release v0.2.6` / `release v0.2.6`
