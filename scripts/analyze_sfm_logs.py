#!/usr/bin/env python3
"""
Analyze InsightAT SfM logs and compare two runs.

Usage:
  python scripts/analyze_sfm_logs.py \
    --baseline /path/to/sfm_cuda_sparse.log \
    --candidate /path/to/sfm_cuda_sparse_reduce_track.log
"""

from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


TIMING_RE = re.compile(r"\[timing\]\s+([A-Za-z0-9_./-]+):\s+([0-9.]+)\s*ms")
PERF_RE = re.compile(r"\[PERF\]\s+([A-Za-z0-9_./-]+):\s*([0-9.]+)\s*ms")
GLOBAL_BA_RMSE_RE = re.compile(
    r"global_bundle_analytic: RMSE\s+before=([0-9.eE+-]+)\s+px\s+after=([0-9.eE+-]+)\s+px\s+iters=([0-9]+)"
)
RUN_GLOBAL_BA_SIZE_RE = re.compile(
    r"run_global_ba:\s+([0-9]+)\s+images,\s+([0-9]+)\s+points.*?,\s+([0-9]+)\s+obs"
)
RUN_LOCAL_BA_SIZE_RE = re.compile(
    r"run_local_ba(?:_colmap|_batch_neighbor)?:.*?obs=([0-9]+)"
)
BA_SAMPLING_RE = re.compile(
    r"BA obs sampling\(([^)]+)\):\s+tracks=([0-9]+).*?before=([0-9]+)\s+after=([0-9]+)\s+"
    r"kept_ratio=([0-9.]+)%"
    r"(?:\s+required_before=([0-9]+)\s+required_after=([0-9]+)\s+required_kept_ratio=([0-9.]+)%)?"
)


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def median(values: List[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    n = len(s)
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


def p95(values: List[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    idx = min(len(s) - 1, math.ceil(0.95 * len(s)) - 1)
    return s[idx]


@dataclass
class LogStats:
    path: str
    timing_ms: Dict[str, List[float]] = field(default_factory=dict)
    perf_ms: Dict[str, List[float]] = field(default_factory=dict)
    global_ba_iters: List[int] = field(default_factory=list)
    global_ba_rmse_before: List[float] = field(default_factory=list)
    global_ba_rmse_after: List[float] = field(default_factory=list)
    run_global_ba_obs: List[int] = field(default_factory=list)
    run_global_ba_points: List[int] = field(default_factory=list)
    run_global_ba_images: List[int] = field(default_factory=list)
    run_local_ba_obs: List[int] = field(default_factory=list)
    sampling: Dict[str, Dict[str, List[float]]] = field(default_factory=dict)

    def add_sampling(
        self,
        name: str,
        tracks: float,
        before: float,
        after: float,
        kept_ratio: float,
        req_before: Optional[float],
        req_after: Optional[float],
        req_kept: Optional[float],
    ) -> None:
        bucket = self.sampling.setdefault(
            name,
            {
                "tracks": [],
                "before": [],
                "after": [],
                "kept_ratio": [],
                "required_before": [],
                "required_after": [],
                "required_kept_ratio": [],
            },
        )
        bucket["tracks"].append(tracks)
        bucket["before"].append(before)
        bucket["after"].append(after)
        bucket["kept_ratio"].append(kept_ratio)
        if req_before is not None:
            bucket["required_before"].append(req_before)
        if req_after is not None:
            bucket["required_after"].append(req_after)
        if req_kept is not None:
            bucket["required_kept_ratio"].append(req_kept)


def parse_log(path: Path) -> LogStats:
    stats = LogStats(path=str(path))
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = TIMING_RE.search(line)
            if m:
                stats.timing_ms.setdefault(m.group(1), []).append(float(m.group(2)))
                continue

            m = PERF_RE.search(line)
            if m:
                stats.perf_ms.setdefault(m.group(1), []).append(float(m.group(2)))
                continue

            m = GLOBAL_BA_RMSE_RE.search(line)
            if m:
                stats.global_ba_rmse_before.append(float(m.group(1)))
                stats.global_ba_rmse_after.append(float(m.group(2)))
                stats.global_ba_iters.append(int(m.group(3)))
                continue

            m = RUN_GLOBAL_BA_SIZE_RE.search(line)
            if m:
                stats.run_global_ba_images.append(int(m.group(1)))
                stats.run_global_ba_points.append(int(m.group(2)))
                stats.run_global_ba_obs.append(int(m.group(3)))
                continue

            m = RUN_LOCAL_BA_SIZE_RE.search(line)
            if m:
                stats.run_local_ba_obs.append(int(m.group(1)))
                continue

            m = BA_SAMPLING_RE.search(line)
            if m:
                stats.add_sampling(
                    name=m.group(1),
                    tracks=float(m.group(2)),
                    before=float(m.group(3)),
                    after=float(m.group(4)),
                    kept_ratio=float(m.group(5)),
                    req_before=float(m.group(6)) if m.group(6) else None,
                    req_after=float(m.group(7)) if m.group(7) else None,
                    req_kept=float(m.group(8)) if m.group(8) else None,
                )
                continue
    return stats


def sum_metric(bucket: Dict[str, List[float]], key: str) -> float:
    return sum(bucket.get(key, []))


def fmt(v: float, digits: int = 2) -> str:
    return f"{v:.{digits}f}"


def render_single(stats: LogStats) -> str:
    lines: List[str] = []
    lines.append(f"## Log: {stats.path}")
    total_timing = sum(sum(vs) for vs in stats.timing_ms.values())
    total_perf = sum(sum(vs) for vs in stats.perf_ms.values())
    lines.append(f"- [timing] total: {fmt(total_timing)} ms")
    lines.append(f"- [PERF] total: {fmt(total_perf)} ms")
    lines.append(
        f"- global_bundle_analytic calls: {len(stats.global_ba_iters)}"
        f", iters mean/median/p95 = {fmt(mean([float(x) for x in stats.global_ba_iters]))}/"
        f"{fmt(median([float(x) for x in stats.global_ba_iters]))}/"
        f"{fmt(p95([float(x) for x in stats.global_ba_iters]))}"
    )
    if stats.run_global_ba_obs:
        lines.append(
            f"- run_global_ba obs mean/median/p95 = {fmt(mean([float(x) for x in stats.run_global_ba_obs]))}/"
            f"{fmt(median([float(x) for x in stats.run_global_ba_obs]))}/"
            f"{fmt(p95([float(x) for x in stats.run_global_ba_obs]))}"
        )
    if stats.sampling:
        lines.append("- sampling summary:")
        for name, bucket in sorted(stats.sampling.items()):
            before = sum_metric(bucket, "before")
            after = sum_metric(bucket, "after")
            kept = (100.0 * after / before) if before > 0 else 0.0
            lines.append(
                f"  - {name}: calls={len(bucket['before'])}, before={int(before)}, after={int(after)}, kept={fmt(kept)}%"
            )
            req_before = sum_metric(bucket, "required_before")
            req_after = sum_metric(bucket, "required_after")
            if req_before > 0:
                req_kept = 100.0 * req_after / req_before
                lines.append(
                    f"    required_before={int(req_before)}, required_after={int(req_after)}, required_kept={fmt(req_kept)}%"
                )
    return "\n".join(lines)


def compare_metric(a: float, b: float) -> Tuple[float, float]:
    delta = b - a
    ratio = (b / a * 100.0) if a != 0 else 0.0
    return delta, ratio


def render_compare(base: LogStats, cand: LogStats) -> str:
    lines: List[str] = []
    lines.append("## Compare (candidate vs baseline)")
    base_timing = sum(sum(vs) for vs in base.timing_ms.values())
    cand_timing = sum(sum(vs) for vs in cand.timing_ms.values())
    d, r = compare_metric(base_timing, cand_timing)
    lines.append(
        f"- [timing] total: baseline={fmt(base_timing)} ms, candidate={fmt(cand_timing)} ms, delta={fmt(d)} ms ({fmt(r)}% of baseline)"
    )

    base_perf = sum(sum(vs) for vs in base.perf_ms.values())
    cand_perf = sum(sum(vs) for vs in cand.perf_ms.values())
    d, r = compare_metric(base_perf, cand_perf)
    lines.append(
        f"- [PERF] total: baseline={fmt(base_perf)} ms, candidate={fmt(cand_perf)} ms, delta={fmt(d)} ms ({fmt(r)}% of baseline)"
    )

    lines.append(
        f"- global BA calls: baseline={len(base.global_ba_iters)}, candidate={len(cand.global_ba_iters)}"
    )
    lines.append(
        f"- global BA iters mean: baseline={fmt(mean([float(x) for x in base.global_ba_iters]))}, "
        f"candidate={fmt(mean([float(x) for x in cand.global_ba_iters]))}"
    )
    if base.run_global_ba_obs and cand.run_global_ba_obs:
        lines.append(
            f"- run_global_ba obs mean: baseline={fmt(mean([float(x) for x in base.run_global_ba_obs]))}, "
            f"candidate={fmt(mean([float(x) for x in cand.run_global_ba_obs]))}"
        )

    names = sorted(set(base.sampling.keys()) | set(cand.sampling.keys()))
    if names:
        lines.append("- sampling compare:")
        for name in names:
            bb = base.sampling.get(name, {})
            cc = cand.sampling.get(name, {})
            b_before = sum(bb.get("before", []))
            b_after = sum(bb.get("after", []))
            c_before = sum(cc.get("before", []))
            c_after = sum(cc.get("after", []))
            b_kept = (100.0 * b_after / b_before) if b_before > 0 else None
            c_kept = (100.0 * c_after / c_before) if c_before > 0 else None
            b_kept_str = f"{fmt(b_kept)}%" if b_kept is not None else "N/A"
            c_kept_str = f"{fmt(c_kept)}%" if c_kept is not None else "N/A"
            lines.append(
                f"  - {name}: baseline kept={b_kept_str} ({int(b_after)}/{int(b_before)}), "
                f"candidate kept={c_kept_str} ({int(c_after)}/{int(c_before)})"
            )
            b_req_before = sum(bb.get("required_before", []))
            b_req_after = sum(bb.get("required_after", []))
            c_req_before = sum(cc.get("required_before", []))
            c_req_after = sum(cc.get("required_after", []))
            if b_req_before > 0 or c_req_before > 0:
                b_req_kept = (100.0 * b_req_after / b_req_before) if b_req_before > 0 else None
                c_req_kept = (100.0 * c_req_after / c_req_before) if c_req_before > 0 else None
                b_req_kept_str = f"{fmt(b_req_kept)}%" if b_req_kept is not None else "N/A"
                c_req_kept_str = f"{fmt(c_req_kept)}%" if c_req_kept is not None else "N/A"
                lines.append(
                    f"    required kept: baseline={b_req_kept_str} ({int(b_req_after)}/{int(b_req_before)}), "
                    f"candidate={c_req_kept_str} ({int(c_req_after)}/{int(c_req_before)})"
                )

    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze and compare InsightAT SfM logs.")
    parser.add_argument("--baseline", required=True, help="Baseline log path.")
    parser.add_argument("--candidate", help="Candidate log path for comparison.")
    args = parser.parse_args()

    baseline = parse_log(Path(args.baseline))
    print(render_single(baseline))
    if args.candidate:
        candidate = parse_log(Path(args.candidate))
        print()
        print(render_single(candidate))
        print()
        print(render_compare(baseline, candidate))


if __name__ == "__main__":
    main()
