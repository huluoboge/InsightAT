'use strict';

const fs = require('fs');
const path = require('path');

/**
 * Poll work/logs/run_* files (console.log, detail.log, events.ndjson) like tail -f.
 * Always follows logs/current.json so a re-run switches to the new run_* directory.
 */
function createLogTailer({ onConsole, onDetail, onProgress, onRunDir, onRunSwitch, intervalMs = 300 }) {
  let timer = null;
  let workDir = '';
  let runDir = '';
  let runId = '';
  let offsets = { console: 0, detail: 0, events: 0 };
  let eventsBuf = '';
  let stopped = true;
  let view = 'console';
  /** Ignore this run_id until current.json points at a newer run (re-run). */
  let baselineRunId = '';
  let waitingForNewRun = false;
  let waitedNotice = false;

  function resolveCurrent() {
    const currentPath = path.join(workDir, 'logs', 'current.json');
    if (!fs.existsSync(currentPath)) return null;
    try {
      const cur = JSON.parse(fs.readFileSync(currentPath, 'utf8'));
      const rel = cur.dir || '';
      const abs = path.isAbsolute(rel) ? rel : path.join(workDir, rel);
      if (!fs.existsSync(abs)) return null;
      return { runId: cur.run_id || path.basename(abs), dir: abs };
    } catch (_) {
      return null;
    }
  }

  function attachRun(next, { announce } = {}) {
    const prev = runDir;
    runDir = next.dir;
    runId = next.runId || '';
    offsets = { console: 0, detail: 0, events: 0 };
    eventsBuf = '';
    if (onRunDir) onRunDir(runDir);
    if (announce && onRunSwitch) {
      onRunSwitch({ runId, dir: runDir, previousDir: prev || null });
    }
  }

  function readGrowth(filePath, key) {
    if (!fs.existsSync(filePath)) return '';
    const st = fs.statSync(filePath);
    const size = st.size;
    if (size < offsets[key]) offsets[key] = 0;
    if (size === offsets[key]) return '';
    const fd = fs.openSync(filePath, 'r');
    try {
      const len = size - offsets[key];
      const buf = Buffer.alloc(len);
      fs.readSync(fd, buf, 0, len, offsets[key]);
      offsets[key] = size;
      return buf.toString('utf8');
    } finally {
      fs.closeSync(fd);
    }
  }

  function consumeEvents(chunk) {
    if (!chunk) return;
    eventsBuf += chunk;
    const lines = eventsBuf.split('\n');
    eventsBuf = lines.pop() || '';
    for (const line of lines) {
      const trimmed = line.trim();
      if (!trimmed) continue;
      let ev;
      try {
        ev = JSON.parse(trimmed);
      } catch (_) {
        continue;
      }
      const type = ev && ev.type;
      if (type === 'progress' && ev.data) {
        if (onProgress) onProgress(ev.data);
      } else if (type === 'step.start' && ev.data && onProgress) {
        onProgress({
          step: ev.data.step,
          step_index: ev.data.step_index,
          step_count: ev.data.step_count,
          fraction: 0,
          overall:
            ev.data.step_count > 0
              ? (ev.data.step_index - 1) / ev.data.step_count
              : 0,
          message: `Starting ${ev.data.step || ''}`
        });
      } else if (type === 'pipeline.end' && onProgress) {
        onProgress({
          overall: 1,
          fraction: 1,
          message: 'Pipeline complete',
          done: true
        });
      } else if (type === 'pipeline.start' && ev.data) {
        const logDir = ev.data.log_dir;
        if (logDir && fs.existsSync(logDir) && logDir !== runDir) {
          waitingForNewRun = false;
          attachRun({ runId: ev.data.run_id || path.basename(logDir), dir: logDir }, { announce: true });
        }
        if (onProgress) {
          onProgress({
            overall: 0,
            fraction: 0,
            message: 'Pipeline started',
            step_count: Array.isArray(ev.data.steps) ? ev.data.steps.length : undefined
          });
        }
      }
    }
  }

  function tick() {
    if (stopped) return;

    const cur = resolveCurrent();
    if (waitingForNewRun) {
      if (!cur || (baselineRunId && cur.runId === baselineRunId)) {
        if (!waitedNotice && onConsole) {
          waitedNotice = true;
          onConsole('# Waiting for new run log (logs/current.json)…\n');
        }
        return;
      }
      waitingForNewRun = false;
      attachRun(cur, { announce: true });
    } else if (!runDir) {
      if (!cur) return;
      // First attach: if we had a baseline and it matches, still wait for a new run.
      if (baselineRunId && cur.runId === baselineRunId) {
        waitingForNewRun = true;
        return;
      }
      attachRun(cur, { announce: true });
    } else if (cur && cur.dir !== runDir) {
      attachRun(cur, { announce: true });
    }

    if (!runDir) return;

    const consoleChunk = readGrowth(path.join(runDir, 'console.log'), 'console');
    const detailChunk = readGrowth(path.join(runDir, 'detail.log'), 'detail');
    const eventsChunk = readGrowth(path.join(runDir, 'events.ndjson'), 'events');
    if (consoleChunk && onConsole) onConsole(consoleChunk);
    if (detailChunk && onDetail) onDetail(detailChunk);
    consumeEvents(eventsChunk);
  }

  return {
    start(nextWorkDir, options = {}) {
      this.stop();
      workDir = nextWorkDir || '';
      runDir = '';
      runId = '';
      offsets = { console: 0, detail: 0, events: 0 };
      eventsBuf = '';
      waitedNotice = false;
      const existing = resolveCurrent();
      baselineRunId = options.baselineRunId != null
        ? options.baselineRunId
        : (existing && existing.runId) || '';
      // Re-run / any start with an existing current.json: wait for a new run_id.
      waitingForNewRun = Boolean(options.waitForNewRun != null
        ? options.waitForNewRun
        : baselineRunId);
      stopped = false;
      timer = setInterval(tick, intervalMs);
      tick();
    },
    stop() {
      if (!stopped) {
        try {
          tick();
        } catch (_) {
          /* ignore */
        }
      }
      stopped = true;
      if (timer) {
        clearInterval(timer);
        timer = null;
      }
    },
    setView(next) {
      view = next === 'detail' ? 'detail' : 'console';
    },
    getView() {
      return view;
    },
    getRunDir() {
      return runDir;
    },
    getRunId() {
      return runId;
    }
  };
}

function readCurrentLogDir(workDir) {
  try {
    const currentPath = path.join(workDir, 'logs', 'current.json');
    if (!fs.existsSync(currentPath)) return null;
    const cur = JSON.parse(fs.readFileSync(currentPath, 'utf8'));
    const abs = path.isAbsolute(cur.dir) ? cur.dir : path.join(workDir, cur.dir);
    if (!fs.existsSync(abs)) return null;
    return { runId: cur.run_id || '', dir: abs };
  } catch (_) {
    return null;
  }
}

module.exports = {
  createLogTailer,
  readCurrentLogDir
};
