#!/usr/bin/env python3
"""Compare exhaustive SiftGPU/PopSift matching on the FIORD tail-image pairs.

This is a diagnostic test. It never changes the project's normal match/geo
directories; all temporary pair lists and outputs are written below --out.

Example:
  python3 scripts/compare_exhaustive_matches.py \
      --project /home/recon/data/insight-prj/fiord \
      --build-dir /home/recon/Git/04jones/InsightAT/build \
      --out /home/recon/data/insight-prj/fiord/exhaustive_match_compare
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


# image indices for:
# 781-783, 781-784, 784-786, 784-787, 786-787
PAIRS = [(197, 200), (195, 197), (195, 198), (195, 199), (198, 199)]


def run(cmd: list[str], log_path: Path) -> None:
    print("$", " ".join(cmd), flush=True)
    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
    if proc.returncode != 0:
        print(f"command failed ({proc.returncode}), see {log_path}", file=sys.stderr)
        raise SystemExit(proc.returncode)


def read_idc_header(path: Path) -> dict:
    import struct

    with path.open("rb") as f:
        if f.read(4) != b"ISAT":
            raise ValueError(f"invalid IDC magic: {path}")
        f.read(4)
        size = struct.unpack("<Q", f.read(8))[0]
        return json.loads(f.read(size).decode("utf-8"))


def write_pairs(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "pairs": [
                    {"image1_index": i, "image2_index": j} for i, j in PAIRS
                ]
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


def collect_rows(match_dir: Path, geo_dir: Path, backend: str) -> list[dict]:
    rows = []
    for i, j in PAIRS:
        match_header = read_idc_header(match_dir / f"{i}_{j}.isat_match")
        geo_header = read_idc_header(geo_dir / f"{i}_{j}.isat_geo")
        rows.append(
            {
                "backend": backend,
                "pair": f"{i}-{j}",
                "raw_matches": match_header.get("metadata", {}).get(
                    "num_matches", match_header.get("num_matches_input", 0)
                ),
                "F_inliers": geo_header.get("geometry", {})
                .get("F", {})
                .get("num_inliers", 0),
                "E_inliers": geo_header.get("geometry", {})
                .get("E", {})
                .get("num_inliers", 0),
            }
        )
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--backend",
        choices=("siftgpu", "popsift", "both"),
        default="both",
        help="exhaustive matcher backend(s), default: both",
    )
    args = parser.parse_args()

    project = args.project
    build = args.build_dir
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    pairs_json = out / "pairs.json"
    write_pairs(pairs_json)

    backends = ("siftgpu", "popsift") if args.backend == "both" else (args.backend,)
    all_rows: list[dict] = []
    for backend in backends:
        match_dir = out / f"match_{backend}"
        geo_dir = out / f"geo_{backend}"
        match_dir.mkdir(parents=True, exist_ok=True)
        geo_dir.mkdir(parents=True, exist_ok=True)

        match_cmd = [
            str(build / "isat_match"),
            "-i",
            str(pairs_json),
            "-f",
            str(project / "feat"),
            "-o",
            str(match_dir),
            "--max-features",
            "-1",
            "--ratio",
            "0.8",
            "--threads",
            "5",
            "--match-backend",
            "cuda",
            "--use-sift-gpu" if backend == "siftgpu" else "--use-pop-sift",
            "-v",
        ]
        run(match_cmd, out / f"match_{backend}.log")

        geo_cmd = [
            str(build / "isat_geo_cuda"),
            "-i",
            str(pairs_json),
            "-m",
            str(match_dir),
            "-o",
            str(geo_dir),
            "-l",
            str(project / "images_all.json"),
            "-t",
            "16",
            "--min-inliers",
            "6",
            "--output-format",
            "geo",
            "--iterations",
            "2000",
            "--threads",
            "5",
            "-v",
        ]
        run(geo_cmd, out / f"geo_{backend}.log")
        all_rows.extend(collect_rows(match_dir, geo_dir, backend))

    result = {"pairs": PAIRS, "results": all_rows}
    (out / "result.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    print("\nbackend pair raw F E")
    for row in all_rows:
        print(
            f"{row['backend']:8s} {row['pair']:7s} "
            f"{row['raw_matches']:3d} {row['F_inliers']:4d} {row['E_inliers']:4d}"
        )
    print(f"\nWrote {out / 'result.json'}")


if __name__ == "__main__":
    main()
