# InsightAT Simple GUI

This is a small Electron shell for the existing InsightAT CLI pipeline. It keeps the user workflow simple:

1. Create or open a work directory.
2. Add one or more image folders.
3. Let the CLI create groups and estimate camera intrinsics.
4. Run SfM reconstruction.

The GUI stores its state in:

```text
<work-dir>/insightat-simple-project.json
```

The real project remains the CLI project:

```text
<work-dir>/project.iat
```

## Run

From this directory:

```bash
npm install
npm start
```

If the CLI tools are not on `PATH`, set `ISAT_BIN_DIR` before starting:

```bash
export ISAT_BIN_DIR=/path/to/InsightAT/build-ceres-12.8
npm start
```

During local development from this repository, the app also probes common build directories such as `build-ceres-12.8`.
