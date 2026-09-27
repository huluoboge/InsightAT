#!/usr/bin/env python3
"""Smoke-check optional colors blob in .isat_feat (IDC).

Usage:
  python3 scripts/check_feat_colors.py path/to/0.isat_feat
  python3 scripts/check_feat_colors.py path/to/feat_dir/   # checks first *.isat_feat
"""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

import numpy as np

_DTYPE_MAP = {
    "float32": np.float32,
    "uint8": np.uint8,
}


def read_idc(path: str):
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"ISAT":
            raise ValueError(f"bad magic {magic!r}")
        _ver = struct.unpack("<I", f.read(4))[0]
        jsz = struct.unpack("<Q", f.read(8))[0]
        hdr = json.loads(f.read(jsz).decode("utf-8"))
        pos = 16 + jsz
        pad = (8 - pos % 8) % 8
        f.read(pad)
        payload = f.read()
    blobs = {}
    for b in hdr.get("blobs", []):
        dtype = _DTYPE_MAP.get(b["dtype"], np.float32)
        arr = np.frombuffer(payload[b["offset"] : b["offset"] + b["size"]], dtype=dtype)
        blobs[b["name"]] = arr.reshape(b["shape"])
    return hdr, blobs


def check_one(path: Path) -> int:
    hdr, blobs = read_idc(str(path))
    kp = blobs.get("keypoints")
    if kp is None:
        print(f"FAIL {path}: missing keypoints")
        return 1
    n = kp.shape[0]
    has_meta = bool(hdr.get("has_colors", False))
    colors = blobs.get("colors")
    if colors is None:
        print(f"OK   {path}: N={n} colors=absent has_colors={has_meta}")
        return 0
    if colors.dtype != np.uint8 or colors.shape != (n, 3):
        print(f"FAIL {path}: colors shape/dtype {colors.shape} {colors.dtype} expected ({n}, 3) uint8")
        return 1
    if has_meta is False:
        print(f"WARN {path}: colors present but has_colors metadata is false")
    print(f"OK   {path}: N={n} colors=uint8[{n},3] mean_rgb={colors.mean(axis=0).astype(int).tolist()}")
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__.strip(), file=sys.stderr)
        return 2
    p = Path(sys.argv[1])
    if p.is_dir():
        files = sorted(p.glob("*.isat_feat"))
        if not files:
            print(f"FAIL no .isat_feat in {p}", file=sys.stderr)
            return 1
        return check_one(files[0])
    return check_one(p)


if __name__ == "__main__":
    sys.exit(main())
