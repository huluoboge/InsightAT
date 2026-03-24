"""
runner.py
InsightAT Pipeline – subprocess runner and event parser.

Provides:
    run_tool(cmd)        – run an isat_* binary, return parsed ISAT_EVENT list
    ToolError            – raised on non-zero exit codes
    find_binary(name)    – locate isat_* executable relative to build/ or ISAT_BIN_DIR
    PipelineEvent        – emit PIPELINE_EVENT lines to stdout
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)

# When True, subprocess stderr is logged at INFO (so it appears in --log-file and console with -v)
_log_tool_stderr_at_info = False
# When True, only stderr lines that look like progress (e.g. "50/550", "Processing") are logged at INFO
_log_tool_stderr_progress_only = False
# When set, run_tool() injects --log-level=<value> into every isat_* command (aligned with CLI_IO_CONVENTIONS).
_cli_log_level: str | None = None


def set_cli_log_level(level: str | None) -> None:
    """Set log level passed to all isat_* subprocesses (error|warn|info|debug). None = do not inject."""
    global _cli_log_level
    _cli_log_level = level


def set_log_tool_stderr_at_info(enabled: bool) -> None:
    """When enabled, run_tool() logs subprocess stderr at INFO (full with -v, or progress-only with --progress)."""
    global _log_tool_stderr_at_info
    _log_tool_stderr_at_info = enabled


def set_log_tool_stderr_progress_only(enabled: bool) -> None:
    """When enabled together with stderr_at_info, only log lines that look like progress (e.g. 10/550, %)."""
    global _log_tool_stderr_progress_only
    _log_tool_stderr_progress_only = enabled


def _looks_like_progress(line: str) -> bool:
    """Heuristic: True if line looks like progress (e.g. '50/550', '45%', 'Processing image')."""
    msg = line.split("] ", 1)[-1].strip() if "] " in line else line.strip()
    if not msg or len(msg) > 200:
        return False
    if "/" in msg and any(c.isdigit() for c in msg):
        return True
    if "%" in msg:
        return True
    for k in ("processing", "extracting", "loading", "encoding", "matching", "done", "complete", "finished", "images", "image "):
        if k in msg.lower():
            return True
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

_ISAT_EVENT_PREFIX = "ISAT_EVENT "
_PIPELINE_EVENT_PREFIX = "PIPELINE_EVENT "


# ─────────────────────────────────────────────────────────────────────────────
# Exceptions
# ─────────────────────────────────────────────────────────────────────────────

class ToolError(Exception):
    """Raised when an isat_* tool exits with a non-zero return code."""

    def __init__(self, tool: str, returncode: int, stderr: str = "") -> None:
        self.tool = tool
        self.returncode = returncode
        self.stderr = stderr
        super().__init__(
            f"{tool} exited with code {returncode}"
            + (f"\nstderr: {stderr[-500:]}" if stderr else "")
        )


# ─────────────────────────────────────────────────────────────────────────────
# Binary locator
# ─────────────────────────────────────────────────────────────────────────────

def _repo_root() -> Path:
    """Return InsightAT repo root (parent of src/pipeline/)."""
    return Path(__file__).resolve().parent.parent.parent


def find_binary(name: str) -> str:
    """
    Locate an isat_* executable.

    Search order:
      1. ISAT_BIN_DIR environment variable
      2. <repo_root>/build/
      3. PATH (via shutil.which)
    """
    # 1. explicit env override
    bin_dir = os.environ.get("ISAT_BIN_DIR", "")
    if bin_dir:
        candidate = Path(bin_dir) / name
        if candidate.is_file():
            return str(candidate)

    # 2. sibling build/ directory
    candidate = _repo_root() / "build" / name
    if candidate.is_file():
        return str(candidate)

    # 3. PATH
    import shutil
    found = shutil.which(name)
    if found:
        return found

    raise FileNotFoundError(
        f"Cannot find binary '{name}'. "
        f"Set ISAT_BIN_DIR or build the project first."
    )


# ─────────────────────────────────────────────────────────────────────────────
# Event helpers
# ─────────────────────────────────────────────────────────────────────────────

def parse_event(line: str) -> dict | None:
    """Parse a single 'ISAT_EVENT <json>' line; return dict or None."""
    line = line.rstrip()
    if not line.startswith(_ISAT_EVENT_PREFIX):
        return None
    payload = line[len(_ISAT_EVENT_PREFIX):]
    try:
        return json.loads(payload)
    except json.JSONDecodeError:
        log.warning("Malformed ISAT_EVENT line: %s", line)
        return None


def emit_pipeline_event(data: dict[str, Any]) -> None:
    """Write a PIPELINE_EVENT line to stdout (machine-readable progress)."""
    print(_PIPELINE_EVENT_PREFIX + json.dumps(data, ensure_ascii=False), flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# Core runner
# ─────────────────────────────────────────────────────────────────────────────

def run_tool(
    cmd: list[str],
    *,
    check: bool = True,
    capture_stderr: bool = True,
) -> list[dict]:
    """
    Run an isat_* tool and return all parsed ISAT_EVENT dicts from stdout.

    Parameters
    ----------
    cmd:
        Full command list, e.g. ['build/isat_project', 'create', '-p', 'x.iat'].
        The first element is the executable path (use find_binary() to resolve).
    check:
        If True (default), raise ToolError on non-zero exit codes.
    capture_stderr:
        If True (default), capture stderr into ToolError; otherwise stream it.

    Returns
    -------
    list[dict]
        All ISAT_EVENT objects emitted by the tool (may be empty).

    Raises
    ------
    ToolError
        When check=True and the process returns non-zero.
    """
    cmd = list(cmd)
    if _cli_log_level:
        tool_name = Path(cmd[0]).name
        if tool_name == "isat_project" and len(cmd) >= 2:
            # isat_project has subcommands (create, add-group, ...); inject after subcommand
            cmd = [cmd[0], cmd[1], "--log-level", _cli_log_level] + cmd[2:]
        else:
            cmd = [cmd[0], "--log-level", _cli_log_level] + cmd[1:]
    tool_name = Path(cmd[0]).name
    log.debug("run_tool: %s", " ".join(str(c) for c in cmd))
    # Optional debug: print the exact command before launching subprocess when
    # ISAT_PRINT_CMD environment variable is set. Useful for debugging binary
    # invocation and arguments from the pipeline.
    print("CMD:", " ".join(str(c) for c in cmd), flush=True)
    start_time = time.perf_counter()
    log.info("Running %s ...", tool_name)

    stderr_dest = subprocess.PIPE if capture_stderr else None

    proc = subprocess.Popen(
        [str(c) for c in cmd],
        stdout=subprocess.PIPE,
        stderr=stderr_dest,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    events: list[dict] = []
    stderr_lines: list[str] = []

    # Start stderr reader thread first so CLI stderr is shown and logged in real time
    stderr_thread = None
    if capture_stderr and proc.stderr:
        def read_stderr():
            for line in proc.stderr:
                stripped = line.rstrip()
                stderr_lines.append(stripped)
                if _log_tool_stderr_at_info:
                    if _log_tool_stderr_progress_only:
                        if _looks_like_progress(stripped):
                            print(stripped, file=sys.stderr, flush=True)
                    else:
                        print(stripped, file=sys.stderr, flush=True)
                else:
                    log.debug("[%s stderr] %s", tool_name, stripped)

        stderr_thread = threading.Thread(target=read_stderr, daemon=True)
        stderr_thread.start()

    # Stream stdout line by line (main thread)
    assert proc.stdout is not None
    for line in proc.stdout:
        stripped = line.rstrip()
        log.debug("[%s stdout] %s", tool_name, stripped)
        evt = parse_event(stripped)
        if evt is not None:
            events.append(evt)

    if stderr_thread is not None:
        stderr_thread.join()

    proc.wait()
    elapsed = time.perf_counter() - start_time
    log.info("%s finished in %.1fs", tool_name, elapsed)

    if check and proc.returncode != 0:
        raise ToolError(
            tool=tool_name,
            returncode=proc.returncode,
            stderr="\n".join(stderr_lines),
        )

    return events
