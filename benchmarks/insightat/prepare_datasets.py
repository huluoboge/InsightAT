#!/usr/bin/env python3
"""
将自定义多视图像数据整理为与 ETH3D 准备脚本一致的目录约定，便于
`benchmarks/sfm_compare/run_colmap_batch.py` / `run_insightat_batch.py` 批跑。

数据集根目录下约定::

  raw/<project>/          # 每个工程一个子目录
    images/               # 输入根：单相机平铺，或多相机时每个相机一个子文件夹
    gt/                   # 可选

  scenes/<project>/
    images/               # 符号链接 → raw/<project>/images（或解析结果）
    gt/                   # 可选 → raw/<project>/gt
    results/colmap/
    results/insightat/

特例：个别工程使用 ``image/``（单数）代替 ``images/``，会自动识别。
特例：无 ``images/`` 但项目根下多个子目录各含图像，则 ``images`` 指向 ``raw/<project>`` 整目录。

用法::

  python3 prepare_custom_datasets.py -d /path/to/dataset_root
  python3 prepare_custom_datasets.py -d /path/to/dataset_root --dry-run
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Iterable, Optional

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp"}


def _is_image(p: Path) -> bool:
    return p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES


def _dir_has_images_directly(d: Path) -> bool:
    if not d.is_dir():
        return False
    try:
        return any(_is_image(x) for x in d.iterdir() if x.is_file())
    except OSError:
        return False


def resolve_raw_project_images_dir(project_dir: Path) -> Optional[tuple[Path, str]]:
    """给定 raw/<project>/，返回 (应对接到 scenes/.../images 的目录, 标签)。"""
    project_dir = project_dir.resolve()
    images = project_dir / "images"
    if images.is_dir():
        return (images, "images/")
    image_singular = project_dir / "image"
    if image_singular.is_dir():
        return (image_singular, "image/")
    try:
        subs = sorted(
            x for x in project_dir.iterdir()
            if x.is_dir() and not x.name.startswith(".")
        )
    except OSError:
        return None
    camera_like = [s for s in subs if _dir_has_images_directly(s)]
    if len(camera_like) >= 1:
        return (project_dir, "multi-cam-at-root")
    return None


def _log(msg: str) -> None:
    print(f"[prepare-custom] {msg}")


def _make_dir(p: Path, base: Path, dry_run: bool) -> None:
    if p.exists():
        return
    _log(f"  mkdir {p.relative_to(base)}")
    if not dry_run:
        p.mkdir(parents=True, exist_ok=True)


def _symlink(src: Path, dst: Path, base: Path, dry_run: bool) -> None:
    src = src.resolve()
    if not src.is_dir():
        _log(f"  [error] 不是目录: {src}")
        return
    if dst.exists() or dst.is_symlink():
        if dry_run:
            _log(f"  [skip-exists] {dst.relative_to(base)}")
            return
        dst.unlink()
    _log(f"  symlink {dst.relative_to(base)} -> {src}")
    if not dry_run:
        dst.symlink_to(src)


def scenes_from_raw_layout(dataset_root: Path) -> list[dict[str, Any]]:
    """扫描 raw/<project>/，生成 scene 条目（name / images / 可选 gt）。"""
    dataset_root = dataset_root.resolve()
    raw_dir = dataset_root / "raw"
    out: list[dict[str, Any]] = []
    if not raw_dir.is_dir():
        _log(f"[error] 缺少 raw 目录: {raw_dir}")
        return out

    for proj in sorted(raw_dir.iterdir()):
        if not proj.is_dir() or proj.name.startswith("."):
            continue
        resolved = resolve_raw_project_images_dir(proj)
        if not resolved:
            _log(f"[skip] 无法解析图像目录（需要 images/、image/ 或根下多相机子目录）: {proj.name}")
            continue
        src, tag = resolved
        rel_img = src.relative_to(dataset_root)
        entry: dict[str, Any] = {
            "name": proj.name,
            "images": str(rel_img).replace("\\", "/"),
        }
        gt = proj / "gt"
        if gt.is_dir():
            entry["gt"] = str(gt.relative_to(dataset_root)).replace("\\", "/")
        out.append(entry)
        _log(f"  {proj.name}: {tag} -> {entry['images']}")
    return out


def materialize_scenes(dataset_root: Path, scenes: Iterable[dict[str, Any]], dry_run: bool) -> None:
    """根据条目创建 scenes/<name>/、符号链接与 results/ 子目录。"""
    dataset_root = dataset_root.resolve()
    _make_dir(dataset_root / "raw", dataset_root, dry_run)
    scenes_dir = dataset_root / "scenes"
    _make_dir(scenes_dir, dataset_root, dry_run)

    for entry in scenes:
        name = entry.get("name", "").strip()
        img_rel = entry.get("images", "").strip()
        gt_rel = (entry.get("gt") or "").strip()
        if not name or not img_rel:
            _log(f"[skip] 缺少 name 或 images: {entry!r}")
            continue

        img_src = Path(img_rel)
        if not img_src.is_absolute():
            img_src = (dataset_root / img_src).resolve()

        scene_dir = scenes_dir / name
        _make_dir(scene_dir, dataset_root, dry_run)
        _make_dir(scene_dir / "results" / "colmap", dataset_root, dry_run)
        _make_dir(scene_dir / "results" / "insightat", dataset_root, dry_run)

        images_link = scene_dir / "images"
        _symlink(img_src, images_link, dataset_root, dry_run)

        if gt_rel:
            gt_src = Path(gt_rel)
            if not gt_src.is_absolute():
                gt_src = (dataset_root / gt_src).resolve()
            if gt_src.is_dir():
                _symlink(gt_src, scene_dir / "gt", dataset_root, dry_run)
            else:
                _log(f"  [warn] gt 不是目录，跳过: {gt_src}")


def print_summary(dataset_root: Path) -> None:
    dataset_root = dataset_root.resolve()
    raw_dir = dataset_root / "raw"
    scenes_dir = dataset_root / "scenes"
    print()
    print("=" * 60)
    if raw_dir.is_dir():
        n_raw = len([p for p in raw_dir.iterdir() if p.is_dir()])
        print(f"  raw/  （原始数据）: {raw_dir}  子目录数={n_raw}")
    print("=" * 60)
    if not scenes_dir.is_dir():
        return
    print(f"  scenes 目录: {scenes_dir}")
    print("=" * 60)
    for s in sorted(p.name for p in scenes_dir.iterdir() if p.is_dir()):
        sd = scenes_dir / s
        img = sd / "images"
        n = len(list(img.iterdir())) if img.exists() else "?"
        print(f"  {s:<36}  images 项数 ~ {n}")
    print()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="根据 raw/<project>/ 自动生成 scenes/（ETH3D 式布局）"
    )
    ap.add_argument(
        "-d",
        "--dataset-root",
        required=True,
        type=Path,
        help="数据集根目录（内含 raw/）",
    )
    ap.add_argument("--dry-run", action="store_true", help="只打印计划，不创建目录或链接")
    args = ap.parse_args()

    root = args.dataset_root.resolve()
    if not root.is_dir():
        print(f"Error: 不是目录: {root}", file=sys.stderr)
        return 2

    if args.dry_run:
        _log("DRY RUN — 不写入")

    _log("按 raw/ 布局生成 scenes/ …")
    scenes = scenes_from_raw_layout(root)
    if not scenes:
        print("Error: raw/ 下没有可解析的工程目录", file=sys.stderr)
        return 2

    materialize_scenes(root, scenes, args.dry_run)
    if not args.dry_run:
        print_summary(root)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
