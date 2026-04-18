#!/usr/bin/env python3
"""
ETH3D dataset preparation script.

Organizes ETH3D DSLR multi-view datasets into a unified structure:

  <dataset_dir>/scenes/<scene_name>/
    images/          -> symlink to raw/<scene>_dslr_jpg/<scene>/images/dslr_images/
    gt/              -> symlink to raw/<scene>_dslr_jpg/<scene>/dslr_calibration_jpg/
    results/
      colmap/        (created by colmap runner)
      insightat/     (created by insightat runner)

Usage:
  python prepare_datasets.py -d /path/to/eth3d [--extract] [--dry-run]

Options:
  -d, --dataset-dir  Root directory of the ETH3D dataset (contains zip/ and/or raw/)
  --extract          Extract all .7z archives in zip/ into raw/ first
  --dry-run          Print actions without executing
"""

import argparse
import subprocess
import sys
from pathlib import Path


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def log(msg: str):
    print(f"[prepare] {msg}")


def scene_name_from_archive(archive: Path) -> str:
    """
    'courtyard_dslr_jpg.7z' -> 'courtyard'
    'meadow_dslr_jpg.7z'    -> 'meadow'
    """
    stem = archive.stem  # e.g. 'courtyard_dslr_jpg'
    for suffix in ("_dslr_jpg", "_dslr_png", "_dslr"):
        if stem.endswith(suffix):
            return stem[: -len(suffix)]
    return stem


def make_symlink(src: Path, dst: Path, base: Path, dry_run: bool):
    """Create (or replace) an absolute symlink dst -> src."""
    if dst.exists() or dst.is_symlink():
        if dry_run:
            log(f"  [skip-exists] {dst.relative_to(base)}")
            return
        dst.unlink()
    if dry_run:
        log(f"  [symlink] {dst.relative_to(base)} -> {src}")
        return
    dst.symlink_to(src)
    log(f"  symlink  {dst.relative_to(base)} -> {src}")


def make_dir(p: Path, base: Path, dry_run: bool):
    if p.exists():
        return
    log(f"  mkdir {p.relative_to(base)}")
    if not dry_run:
        p.mkdir(parents=True, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# Extraction
# ─────────────────────────────────────────────────────────────────────────────

def extract_archives(base_dir: Path, dry_run: bool):
    zip_dir = base_dir / "zip"
    raw_dir = base_dir / "raw"

    archives = sorted(zip_dir.glob("*.7z"))
    if not archives:
        log(f"No .7z files found in {zip_dir}")
        return

    make_dir(raw_dir, base_dir, dry_run)

    for arch in archives:
        scene = scene_name_from_archive(arch)
        # Accept either wrapper-style (scene_dslr_jpg/) or flat-style (scene/) extraction
        already_extracted = (raw_dir / f"{scene}_dslr_jpg").exists() or (raw_dir / scene).exists()
        if already_extracted:
            log(f"  [skip] already extracted: {scene}")
            continue
        log(f"  extracting {arch.name} -> raw/")
        if not dry_run:
            result = subprocess.run(
                ["7z", "x", str(arch), f"-o{raw_dir}", "-y"],
                capture_output=True, text=True
            )
            if result.returncode != 0:
                print(result.stderr, file=sys.stderr)
                raise RuntimeError(f"Failed to extract {arch.name}")


# ─────────────────────────────────────────────────────────────────────────────
# Organize
# ─────────────────────────────────────────────────────────────────────────────

def _has_scene_layout(d: Path) -> bool:
    """Return True if d directly contains the expected ETH3D scene layout."""
    return (d / "images" / "dslr_images").is_dir() or (d / "dslr_calibration_jpg").is_dir()


def find_raw_roots(base_dir: Path) -> list[tuple[str, Path]]:
    """
    Return list of (scene_name, inner_dir) pairs.
    inner_dir is the directory containing images/dslr_images/ and dslr_calibration_jpg/.

    Handles two layouts:
      A) <pkg_dir>/<scene>/images/dslr_images/   (manually extracted with _dslr_jpg wrapper)
      B) <pkg_dir>/images/dslr_images/           (7z extraction – no wrapper dir)

    Searches BASE_DIR and BASE_DIR/raw/.
    """
    results = []
    raw_dir = base_dir / "raw"
    search_dirs = [base_dir]
    if raw_dir.exists():
        search_dirs.append(raw_dir)

    seen_scenes: set[str] = set()

    for search in search_dirs:
        for pkg_dir in sorted(search.iterdir()):
            if not pkg_dir.is_dir():
                continue

            # ── Layout B: pkg_dir itself is the inner scene dir ───────────
            if _has_scene_layout(pkg_dir):
                scene = pkg_dir.name
                # strip known suffixes so both "courtyard" and "courtyard_dslr_jpg" → "courtyard"
                for suffix in ("_dslr_jpg", "_dslr_png", "_dslr"):
                    if scene.endswith(suffix):
                        scene = scene[: -len(suffix)]
                        break
                if scene not in seen_scenes:
                    seen_scenes.add(scene)
                    results.append((scene, pkg_dir))
                continue

            # ── Layout A: pkg_dir wraps an inner dir (e.g. *_dslr_jpg/) ──
            if not pkg_dir.name.endswith(("_dslr_jpg", "_dslr_png", "_dslr")):
                continue
            scene = pkg_dir.name
            for suffix in ("_dslr_jpg", "_dslr_png", "_dslr"):
                if scene.endswith(suffix):
                    scene = scene[: -len(suffix)]
                    break
            if scene in seen_scenes:
                continue
            inner = pkg_dir / scene
            if not inner.is_dir():
                subdirs = [d for d in pkg_dir.iterdir() if d.is_dir()]
                if len(subdirs) == 1:
                    inner = subdirs[0]
                else:
                    log(f"  [warn] cannot locate inner dir in {pkg_dir.name}, skipping")
                    continue
            seen_scenes.add(scene)
            results.append((scene, inner))

    return results


def organize_scene(scene: str, inner: Path, base_dir: Path, dry_run: bool):
    scenes_dir = base_dir / "scenes"
    scene_dir = scenes_dir / scene
    make_dir(scene_dir, base_dir, dry_run)
    make_dir(scene_dir / "results" / "colmap", base_dir, dry_run)
    make_dir(scene_dir / "results" / "insightat", base_dir, dry_run)

    images_src = inner / "images" / "dslr_images"
    if images_src.exists():
        make_symlink(images_src, scene_dir / "images", base_dir, dry_run)
    else:
        log(f"  [warn] images dir not found: {images_src}")

    gt_src = inner / "dslr_calibration_jpg"
    if gt_src.exists():
        make_symlink(gt_src, scene_dir / "gt", base_dir, dry_run)
    else:
        log(f"  [warn] gt dir not found: {gt_src}")


def organize_all(base_dir: Path, dry_run: bool):
    scenes_dir = base_dir / "scenes"
    make_dir(scenes_dir, base_dir, dry_run)
    raw_roots = find_raw_roots(base_dir)
    if not raw_roots:
        log("No extracted scene directories found. Run with --extract first, or extract manually.")
        return
    for scene, inner in raw_roots:
        log(f"Organizing scene: {scene}")
        organize_scene(scene, inner, base_dir, dry_run)


# ─────────────────────────────────────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────────────────────────────────────

def print_summary(base_dir: Path):
    scenes_dir = base_dir / "scenes"
    if not scenes_dir.exists():
        log("No scenes/ directory found")
        return
    scenes = sorted(p.name for p in scenes_dir.iterdir() if p.is_dir())
    print()
    print("=" * 60)
    print(f"  ETH3D scenes ready: {len(scenes)}")
    print("=" * 60)
    for s in scenes:
        sd = scenes_dir / s
        img_dir = sd / "images"
        gt_dir  = sd / "gt"
        n_imgs  = len(list(img_dir.iterdir())) if img_dir.exists() else "?"
        has_gt  = "yes" if gt_dir.exists() else "no"
        print(f"  {s:<20}  images={n_imgs:<5}  gt={has_gt}")
    print()
    print("Directory layout:")
    print(f"  {base_dir}/")
    print("    scenes/")
    print("      <scene>/")
    print("        images/       <- symlink to dslr_images/")
    print("        gt/           <- symlink to dslr_calibration_jpg/ (COLMAP GT format)")
    print("        results/")
    print("          colmap/     <- COLMAP reconstruction output")
    print("          insightat/  <- InsightAT reconstruction output")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Prepare ETH3D datasets for benchmarking")
    parser.add_argument("-d", "--dataset-dir", required=True,
                        help="Root directory of the ETH3D dataset (e.g. /data/01-eth3d)")
    parser.add_argument("--extract", action="store_true",
                        help="Extract .7z archives from <dataset_dir>/zip/ into <dataset_dir>/raw/")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print actions without executing")
    args = parser.parse_args()

    base_dir = Path(args.dataset_dir).resolve()
    if not base_dir.exists():
        print(f"Error: dataset dir does not exist: {base_dir}", file=sys.stderr)
        sys.exit(2)

    if args.dry_run:
        log("DRY RUN mode — no files will be created")

    if args.extract:
        log("Step 1: Extracting archives...")
        extract_archives(base_dir, args.dry_run)
    else:
        log("Skipping extraction (pass --extract to extract .7z archives)")

    log("Step 2: Organizing scenes...")
    organize_all(base_dir, args.dry_run)

    if not args.dry_run:
        print_summary(base_dir)


if __name__ == "__main__":
    main()
