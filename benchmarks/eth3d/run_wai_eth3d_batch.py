#!/usr/bin/env python3
"""
Prepare and batch-run InsightAT SfM on the wai_eth3d scene layout.

The source scenes are never modified.  For an input directory such as:

  /home/recon/vrdata2/000_open_datasets/ETH3D/wai_eth3d/
    courtyard/
      images/
      scene_meta.json

Given an output directory such as /home/recon/data/00scene/sfm/eth3d, this script creates:

  /home/recon/data/00scene/sfm/eth3d/
    raw/courtyard -> /data/wai_eth3d/courtyard
    prj/courtyard/
      images -> ../../raw/courtyard/images
      scene_meta.json -> ../../raw/courtyard/scene_meta.json
      work/                         # isat_sfm output
      isat_sfm.log
      run.json
      poses.json                    # GT and estimated poses, matched by image name
    prj/poses.json                  # all-scene pose collection

Usage:
  python3 benchmarks/eth3d/run_wai_eth3d_batch.py \
      /home/recon/vrdata2/000_open_datasets/ETH3D/wai_eth3d \
      /home/recon/data/00scene/sfm/eth3d

Set ISAT_BIN_DIR to select another directory containing isat_sfm.
Extra arguments after ``--`` are forwarded to isat_sfm.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp", ".webp"}


def log(message: str) -> None:
    print(f"[eth3d-batch] {message}", flush=True)


def write_json(path: Path, value: Any) -> None:
    """Atomically replace a generated JSON file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return value


def resolve_isat_sfm(explicit: str) -> str:
    candidates: list[Path] = []
    if explicit:
        candidate = Path(explicit).expanduser()
        candidates.append(candidate / "isat_sfm" if candidate.is_dir() else candidate)
    env_dir = os.environ.get("ISAT_BIN_DIR")
    if env_dir:
        candidates.append(Path(env_dir).expanduser() / "isat_sfm")
    candidates.append(REPO_ROOT / "build" / "isat_sfm")

    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate.resolve())

    from_path = shutil.which("isat_sfm")
    if from_path:
        return from_path
    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(f"找不到可执行文件 isat_sfm；已检查: {searched} 以及 PATH")


def discover_scenes(dataset_root: Path) -> list[tuple[str, Path]]:
    """Discover immediate source scenes without traversing generated raw/prj trees."""
    found: dict[str, Path] = {}

    for child in sorted(dataset_root.iterdir()):
        if child.name in {"raw", "prj"} or not child.is_dir():
            continue
        if (child / "scene_meta.json").is_file():
            found[child.name] = child.resolve()

    # Also accept an already organized dataset whose real source scenes live in raw/.
    raw_root = dataset_root / "raw"
    if raw_root.is_dir():
        for child in sorted(raw_root.iterdir()):
            if child.is_dir() and (child / "scene_meta.json").is_file():
                found.setdefault(child.name, child.resolve())

    return sorted(found.items())


def ensure_symlink(link: Path, target: Path) -> None:
    """Create a relative directory/file symlink, refusing to replace unrelated paths."""
    target = target.absolute()
    canonical_target = target.resolve()
    if link.is_symlink():
        if link.resolve() == canonical_target:
            return
        raise RuntimeError(f"已有软链接指向其他位置，拒绝覆盖: {link} -> {link.resolve()}")
    if link.exists():
        if link.resolve() == canonical_target:
            return
        raise RuntimeError(f"路径已存在且不是目标软链接，拒绝覆盖: {link}")
    link.parent.mkdir(parents=True, exist_ok=True)
    relative_target = os.path.relpath(target, start=link.parent.resolve())
    link.symlink_to(relative_target, target_is_directory=target.is_dir())


def scene_image_dir(scene_source: Path, scene_meta: dict[str, Any]) -> Path:
    frames = scene_meta.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError(f"{scene_source}/scene_meta.json 没有有效 frames")

    image_value = frames[0].get("file_path") or frames[0].get("image")
    if not isinstance(image_value, str) or not image_value:
        raise ValueError(f"{scene_source}/scene_meta.json 的首帧没有图像路径")
    image_dir = (scene_source / Path(image_value).parent).resolve()
    if not image_dir.is_dir():
        raise FileNotFoundError(f"图像目录不存在: {image_dir}")
    if not any(
        item.is_file() and item.suffix.lower() in IMAGE_SUFFIXES for item in image_dir.iterdir()
    ):
        raise ValueError(f"图像目录为空: {image_dir}")
    return image_dir


def prepare_scene(
    output_root: Path,
    scene_name: str,
    scene_source: Path,
) -> tuple[Path, Path, dict[str, Any]]:
    scene_meta_path = scene_source / "scene_meta.json"
    scene_meta = read_json(scene_meta_path)
    source_images = scene_image_dir(scene_source, scene_meta)

    raw_scene = output_root / "raw" / scene_name
    ensure_symlink(raw_scene, scene_source)

    project_scene = output_root / "prj" / scene_name
    project_scene.mkdir(parents=True, exist_ok=True)
    source_image_relative = source_images.relative_to(scene_source)
    ensure_symlink(project_scene / "images", raw_scene / source_image_relative)
    ensure_symlink(project_scene / "scene_meta.json", raw_scene / "scene_meta.json")
    return project_scene, project_scene / "work", scene_meta


def matrix3_transpose(values: list[float]) -> list[list[float]]:
    if len(values) != 9:
        raise ValueError("rotation must contain 9 values")
    return [[values[col * 3 + row] for col in range(3)] for row in range(3)]


def estimated_camera_to_world(rotation: list[float], center: list[float]) -> list[list[float]]:
    # InsightAT pose convention: X_camera = R * (X_world - C).
    rotation_camera_to_world = matrix3_transpose(rotation)
    return [
        rotation_camera_to_world[0] + [center[0]],
        rotation_camera_to_world[1] + [center[1]],
        rotation_camera_to_world[2] + [center[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def image_names_by_index(images_all_path: Path) -> dict[int, str]:
    if not images_all_path.is_file():
        return {}
    images_all = read_json(images_all_path)
    result: dict[int, str] = {}
    images = images_all.get("images", [])
    if not isinstance(images, list):
        return result
    for fallback_index, image in enumerate(images):
        if not isinstance(image, dict):
            continue
        image_index = image.get("image_index", fallback_index)
        image_path = image.get("path")
        if isinstance(image_index, int) and isinstance(image_path, str):
            result[image_index] = Path(image_path).name
    return result


def collect_poses(
    scene_name: str,
    scene_meta: dict[str, Any],
    work_dir: Path,
    exit_code: int | None,
) -> dict[str, Any]:
    gt_poses: list[dict[str, Any]] = []
    frames = scene_meta.get("frames", [])
    for frame in frames if isinstance(frames, list) else []:
        if not isinstance(frame, dict):
            continue
        image_value = frame.get("file_path") or frame.get("image")
        transform = frame.get("transform_matrix")
        if not isinstance(image_value, str) or not isinstance(transform, list):
            continue
        if len(transform) != 4 or any(not isinstance(row, list) or len(row) != 4 for row in transform):
            continue
        gt_poses.append(
            {
                "image_name": Path(image_value).name,
                "frame_name": frame.get("frame_name", Path(image_value).stem),
                "camera_to_world": transform,
                "camera_center": [transform[0][3], transform[1][3], transform[2][3]],
                "intrinsics": {
                    key: frame[key]
                    for key in ("fl_x", "fl_y", "cx", "cy", "w", "h")
                    if key in frame
                },
            }
        )

    estimated: list[dict[str, Any]] = []
    estimate_path = work_dir / "incremental_sfm" / "poses.json"
    names = image_names_by_index(work_dir / "images_all.json")
    estimate_bundle: dict[str, Any] = {}
    if estimate_path.is_file():
        estimate_bundle = read_json(estimate_path)
        cameras = estimate_bundle.get("cameras", [])
        for pose in estimate_bundle.get("poses", []):
            if not isinstance(pose, dict):
                continue
            image_index = pose.get("image_index")
            rotation = pose.get("R")
            center = pose.get("C")
            if (
                not isinstance(image_index, int)
                or not isinstance(rotation, list)
                or len(rotation) != 9
                or not isinstance(center, list)
                or len(center) != 3
            ):
                continue
            camera_index = pose.get("camera_index")
            intrinsics = None
            if isinstance(camera_index, int) and isinstance(cameras, list):
                if 0 <= camera_index < len(cameras):
                    intrinsics = cameras[camera_index]
            estimated.append(
                {
                    "image_name": names.get(image_index, ""),
                    "image_index": image_index,
                    "camera_index": camera_index,
                    "R_world_to_camera": [
                        rotation[0:3],
                        rotation[3:6],
                        rotation[6:9],
                    ],
                    "camera_center": center,
                    "camera_to_world": estimated_camera_to_world(rotation, center),
                    "intrinsics": intrinsics,
                }
            )

    gt_names = {pose["image_name"] for pose in gt_poses}
    estimated_names = {pose["image_name"] for pose in estimated if pose["image_name"]}
    return {
        "schema": "insightat_eth3d_pose_collection_v1",
        "scene": scene_name,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "run_exit_code": exit_code,
        "coordinate_systems": {
            "ground_truth": {
                "pose": "camera_to_world",
                "camera_convention": scene_meta.get("camera_convention", "opencv"),
                "scale": scene_meta.get("scale_type", "metric"),
            },
            "estimated": {
                "pose": "camera_to_world",
                "source_pose": "X_camera = R_world_to_camera * (X_world - camera_center)",
                "scale": "arbitrary",
            },
            "comparison_note": (
                "真值和估计位姿处于独立坐标系；比较前需使用同名相机中心估计 Sim(3) 相似变换。"
            ),
        },
        "ground_truth": gt_poses,
        "estimated": estimated,
        "matching": {
            "ground_truth_count": len(gt_poses),
            "estimated_count": len(estimated),
            "common_image_count": len(gt_names & estimated_names),
            "common_image_names": sorted(gt_names & estimated_names),
        },
        "source_files": {
            "ground_truth": "scene_meta.json",
            "estimated": (
                str(estimate_path.relative_to(work_dir.parent)) if estimate_path.is_file() else ""
            ),
        },
    }


def previous_run_succeeded(run_path: Path) -> bool:
    if not run_path.is_file():
        return False
    try:
        run = read_json(run_path)
    except (OSError, ValueError, json.JSONDecodeError):
        return False
    return run.get("exit_code") == 0


def count_images(directory: Path) -> int:
    return sum(
        1
        for item in directory.iterdir()
        if item.is_file() and item.suffix.lower() in IMAGE_SUFFIXES
    )


def run_scene(
    isat_sfm: str,
    scene_name: str,
    project_scene: Path,
    work_dir: Path,
    force: bool,
    extra_args: Iterable[str],
) -> tuple[int, dict[str, Any]]:
    run_path = project_scene / "run.json"
    if not force and previous_run_succeeded(run_path):
        log(f"{scene_name}: 已完成，跳过（使用 --force 可重跑）")
        return 0, read_json(run_path)

    if work_dir.exists():
        if force:
            shutil.rmtree(work_dir)
        elif any(work_dir.iterdir()):
            raise RuntimeError(
                f"{scene_name}: {work_dir} 已存在但没有成功记录；"
                "为避免覆盖，请检查后使用 --force 重跑"
            )
    work_dir.mkdir(parents=True, exist_ok=True)

    command = [
        isat_sfm,
        "-i",
        str(project_scene / "images"),
        "-w",
        str(work_dir),
        "--undistort",
        "--binary",
    ]
    command.extend(extra_args)
    log_path = project_scene / "isat_sfm.log"
    log(f"{scene_name}: 开始 SfM，日志 {log_path}")
    started_at = datetime.now(timezone.utc)
    start = time.perf_counter()
    with log_path.open("w", encoding="utf-8") as output:
        output.write("$ " + " ".join(command) + "\n\n")
        output.flush()
        completed = subprocess.run(
            command,
            stdout=output,
            stderr=subprocess.STDOUT,
            check=False,
        )
    elapsed = time.perf_counter() - start

    sparse_dir = work_dir / "incremental_sfm" / "colmap" / "sparse" / "0"
    undistorted_dir = work_dir / "incremental_sfm" / "colmap" / "images"
    estimate_path = work_dir / "incremental_sfm" / "poses.json"
    run_record = {
        "scene": scene_name,
        "started_at": started_at.isoformat(),
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "elapsed_wall_s": round(elapsed, 3),
        "exit_code": completed.returncode,
        "command": command,
        "input_images": count_images(project_scene / "images"),
        "outputs": {
            "work_dir": str(work_dir),
            "poses_json": str(estimate_path) if estimate_path.is_file() else "",
            "colmap_sparse_binary": str(sparse_dir),
            "colmap_binary_complete": all(
                (sparse_dir / name).is_file()
                for name in ("cameras.bin", "images.bin", "points3D.bin")
            ),
            "undistorted_images": str(undistorted_dir),
            "undistorted_image_count": (
                count_images(undistorted_dir) if undistorted_dir.is_dir() else 0
            ),
            "log": str(log_path),
        },
    }
    write_json(run_path, run_record)
    log(f"{scene_name}: 结束，exit={completed.returncode}，耗时={elapsed:.1f}s")
    return completed.returncode, run_record


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="以只读软链接组织 wai_eth3d，并逐场景运行 InsightAT SfM"
    )
    parser.add_argument(
        "input_dir",
        type=Path,
        help="只读输入数据集根目录（包含各场景的 scene_meta.json）",
    )
    parser.add_argument(
        "output_dir",
        type=Path,
        help="输出根目录；raw/ 软链接和 prj/ SfM 项目将创建在此目录下",
    )
    parser.add_argument(
        "--isat-bin",
        default="",
        help="isat_sfm 可执行文件或其所在目录（默认查找 ISAT_BIN_DIR、仓库 build、PATH）",
    )
    parser.add_argument(
        "--scene",
        action="append",
        default=[],
        help="只处理指定场景，可重复传入",
    )
    parser.add_argument(
        "--prepare-only",
        action="store_true",
        help="只创建软链接和收集已有位姿，不运行 SfM",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="删除并重建 prj/<scene>/work；不会删除或修改 raw 指向的原始数据",
    )
    args, extra = parser.parse_known_args()
    if extra and extra[0] == "--":
        extra = extra[1:]
    return args, extra


def main() -> int:
    args, extra_args = parse_args()
    source_root = args.input_dir.expanduser().resolve()
    if not source_root.is_dir():
        print(f"错误: 只读数据集目录不存在: {source_root}", file=sys.stderr)
        return 2
    output_root = args.output_dir.expanduser().resolve()
    if output_root == source_root or source_root in output_root.parents:
        print(
            f"错误: 输出目录不得位于只读数据集目录内: {output_root}",
            file=sys.stderr,
        )
        return 2
    output_root.mkdir(parents=True, exist_ok=True)

    scenes = discover_scenes(source_root)
    if args.scene:
        selected = set(args.scene)
        available = {name for name, _ in scenes}
        missing = sorted(selected - available)
        if missing:
            print(f"错误: 未找到场景: {', '.join(missing)}", file=sys.stderr)
            return 2
        scenes = [(name, source) for name, source in scenes if name in selected]
    if not scenes:
        print(f"错误: {source_root} 下未发现包含 scene_meta.json 的场景", file=sys.stderr)
        return 2

    isat_sfm = ""
    if not args.prepare_only:
        try:
            isat_sfm = resolve_isat_sfm(args.isat_bin)
        except FileNotFoundError as error:
            print(f"错误: {error}", file=sys.stderr)
            return 2
        log(f"isat_sfm: {isat_sfm}")

    log(f"只读数据源: {source_root}")
    log(f"输出目录: {output_root}")
    log(f"发现 {len(scenes)} 个场景；原始场景仅以读取方式访问")
    all_poses: list[dict[str, Any]] = []
    batch_rows: list[dict[str, Any]] = []
    failed = 0

    for scene_name, scene_source in scenes:
        try:
            project_scene, work_dir, scene_meta = prepare_scene(
                output_root, scene_name, scene_source
            )
            exit_code: int | None = None
            run_record: dict[str, Any] = {"scene": scene_name, "prepared_only": True}
            if not args.prepare_only:
                exit_code, run_record = run_scene(
                    isat_sfm,
                    scene_name,
                    project_scene,
                    work_dir,
                    args.force,
                    extra_args,
                )
                if exit_code != 0:
                    failed += 1

            poses = collect_poses(scene_name, scene_meta, work_dir, exit_code)
            write_json(project_scene / "poses.json", poses)
            all_poses.append(poses)
            batch_rows.append(run_record)
        except Exception as error:
            failed += 1
            log(f"{scene_name}: 失败: {error}")
            batch_rows.append({"scene": scene_name, "exit_code": -1, "error": str(error)})

    project_root = output_root / "prj"
    write_json(
        project_root / "poses.json",
        {
            "schema": "insightat_eth3d_batch_pose_collection_v1",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "source_root": str(source_root),
            "output_root": str(output_root),
            "scenes": all_poses,
        },
    )
    write_json(
        project_root / "batch_summary.json",
        {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "scene_count": len(scenes),
            "failed_count": failed,
            "runs": batch_rows,
        },
    )
    log(f"汇总位姿: {project_root / 'poses.json'}")
    log(f"批处理摘要: {project_root / 'batch_summary.json'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
