'use strict';

const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawn, spawnSync } = require('child_process');
const { createLogTailer, readCurrentLogDir } = require('./log_tailer');

const STATE_FILE = 'insightat-simple-project.json';
const PROJECT_FILE = 'project.iat';
const DEFAULT_EXT = '.jpg,.jpeg,.tif,.tiff,.png';
const SETTINGS_FILE = 'settings.json';

/** Electron userData dir; set from main via setUserDataDir(). */
let userDataDir = '';

function setUserDataDir(dir) {
  userDataDir = dir ? path.resolve(dir) : '';
}

function settingsPath() {
  const base = userDataDir
    || path.join(os.homedir(), '.config', 'InsightAT', 'sfm-gui');
  return path.join(base, SETTINGS_FILE);
}

function defaultSettings() {
  return {
    binDir: '',
    sfmViewerPath: ''
  };
}

function loadUserSettings() {
  const file = settingsPath();
  try {
    if (!fs.existsSync(file)) return defaultSettings();
    const raw = JSON.parse(fs.readFileSync(file, 'utf8'));
    return {
      binDir: typeof raw.binDir === 'string' ? raw.binDir.trim() : '',
      sfmViewerPath: typeof raw.sfmViewerPath === 'string' ? raw.sfmViewerPath.trim() : ''
    };
  } catch (_) {
    return defaultSettings();
  }
}

function saveUserSettings(partial = {}) {
  const next = {
    ...loadUserSettings(),
    ...partial
  };
  if (typeof next.binDir === 'string') next.binDir = next.binDir.trim();
  else next.binDir = '';
  if (typeof next.sfmViewerPath === 'string') next.sfmViewerPath = next.sfmViewerPath.trim();
  else next.sfmViewerPath = '';

  if (next.binDir && !hasCliBinary(next.binDir)) {
    throw new Error(`CLI tools not found in: ${next.binDir} (need isat_project)`);
  }
  if (next.sfmViewerPath && !isSfmViewerLaunchable(next.sfmViewerPath)) {
    throw new Error(
      `sfm-viewer path invalid: ${next.sfmViewerPath} (need app folder or executable)`
    );
  }

  const file = settingsPath();
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, `${JSON.stringify(next, null, 2)}\n`);
  return next;
}

function normalizeExts(raw) {
  return String(raw || DEFAULT_EXT)
    .split(',')
    .map((item) => item.trim().toLowerCase())
    .filter(Boolean)
    .map((item) => (item.startsWith('.') ? item : `.${item}`))
    .join(',');
}

function extSet(raw) {
  return new Set(normalizeExts(raw).split(',').filter(Boolean));
}

function hasImages(dir, extensions) {
  let entries = [];
  try {
    entries = fs.readdirSync(dir, { withFileTypes: true });
  } catch (_) {
    return false;
  }
  return entries.some((entry) => {
    if (!entry.isFile()) return false;
    return extensions.has(path.extname(entry.name).toLowerCase());
  });
}

function walkDirs(root, out = []) {
  let entries = [];
  try {
    entries = fs.readdirSync(root, { withFileTypes: true });
  } catch (_) {
    return out;
  }
  for (const entry of entries) {
    if (!entry.isDirectory()) continue;
    const fullPath = path.join(root, entry.name);
    out.push(fullPath);
    walkDirs(fullPath, out);
  }
  return out;
}

function safeGroupName(input) {
  return input
    .replace(/[\\/]+/g, '_')
    .replace(/[^a-zA-Z0-9_.-]+/g, '_')
    .replace(/^_+|_+$/g, '')
    .slice(0, 120) || 'images';
}

function scanGroups(rootDir, rawExt) {
  const root = path.resolve(rootDir);
  const extensions = extSet(rawExt);
  const rootBase = safeGroupName(path.basename(root));
  const groups = [];

  for (const dir of walkDirs(root)) {
    if (!hasImages(dir, extensions)) continue;
    const rel = path.relative(root, dir);
    const name = safeGroupName(rel ? `${rootBase}_${rel}` : rootBase);
    groups.push({ name, path: dir });
  }

  if (groups.length === 0 && hasImages(root, extensions)) {
    groups.push({ name: rootBase, path: root });
  }

  groups.sort((a, b) => a.path.localeCompare(b.path));
  return groups;
}

function statePath(workDir) {
  return path.join(workDir, STATE_FILE);
}

function projectPath(workDir) {
  return path.join(workDir, PROJECT_FILE);
}

function defaultState(workDir, overrides = {}) {
  const resolvedWorkDir = path.resolve(workDir);
  return {
    schemaVersion: 1,
    name: overrides.name || path.basename(resolvedWorkDir),
    workDir: resolvedWorkDir,
    projectPath: projectPath(resolvedWorkDir),
    binDir: overrides.binDir || resolveCliBinDir() || '',
    ext: normalizeExts(overrides.ext),
    maxSample: Number.isInteger(overrides.maxSample) ? overrides.maxSample : 5,
    folders: [],
    groups: [],
    cameraMode: 'auto',
    manualCamera: null,
    latestTaskId: null,
    imagesAllPath: path.join(resolvedWorkDir, 'images_all.json'),
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString()
  };
}

function loadState(workDir) {
  const file = statePath(workDir);
  const state = JSON.parse(fs.readFileSync(file, 'utf8'));
  state.workDir = path.resolve(state.workDir || workDir);
  state.projectPath = state.projectPath || projectPath(state.workDir);
  state.imagesAllPath = state.imagesAllPath || path.join(state.workDir, 'images_all.json');
  state.ext = normalizeExts(state.ext);
  state.folders = Array.isArray(state.folders) ? state.folders : [];
  state.groups = Array.isArray(state.groups) ? state.groups : [];
  if (!state.binDir || !hasCliBinary(state.binDir)) {
    state.binDir = resolveCliBinDir() || state.binDir || '';
  }
  if (state.cameraMode !== 'manual' && state.cameraMode !== 'auto') {
    const anyManual = state.groups.some((g) => g.intrinsicsSource === 'manual');
    state.cameraMode = anyManual ? 'manual' : 'auto';
  }
  if (state.cameraMode === 'manual') {
    state.groups = state.groups.map((g) => ({ ...g, intrinsicsSource: 'manual' }));
  } else {
    state.groups = state.groups.map((g) => ({
      ...g,
      intrinsicsSource: 'auto',
      fixIntrinsics: false
    }));
  }
  return state;
}

function saveState(state) {
  const next = { ...state, updatedAt: new Date().toISOString() };
  fs.mkdirSync(next.workDir, { recursive: true });
  fs.writeFileSync(statePath(next.workDir), `${JSON.stringify(next, null, 2)}\n`);
  return next;
}

function packagedBinDir() {
  if (process.resourcesPath) {
    const candidate = path.join(process.resourcesPath, 'bin');
    if (fs.existsSync(candidate)) return candidate;
  }
  return '';
}

function hasCliBinary(dir, exeName = 'isat_project') {
  if (!dir) return false;
  try {
    const base = path.resolve(dir);
    if (fs.existsSync(path.join(base, exeName))) return true;
    if (process.platform === 'win32' && fs.existsSync(path.join(base, `${exeName}.exe`))) {
      return true;
    }
    return false;
  } catch (_) {
    return false;
  }
}

function isSfmViewerAppDir(dir) {
  if (!dir) return false;
  try {
    const base = path.resolve(dir);
    return (
      fs.existsSync(path.join(base, 'package.json')) &&
      fs.existsSync(path.join(base, 'src', 'main.js'))
    );
  } catch (_) {
    return false;
  }
}

function isSfmViewerLaunchable(candidate) {
  if (!candidate) return false;
  try {
    const resolved = path.resolve(candidate);
    if (!fs.existsSync(resolved)) return false;
    const st = fs.statSync(resolved);
    if (st.isFile()) return true;
    return isSfmViewerAppDir(resolved);
  } catch (_) {
    return false;
  }
}

/**
 * Auto-locate InsightAT CLI tools. Prefer user settings, then bundled
 * locations, then system install paths (/usr/lib/insightat/bin, PATH),
 * then repo build dirs.
 */
function resolveCliBinDir() {
  const candidates = [];

  const settings = loadUserSettings();
  if (settings.binDir) candidates.push(settings.binDir);

  const resourceBin = packagedBinDir();
  if (resourceBin) candidates.push(resourceBin);

  if (process.env.ISAT_BIN_DIR) candidates.push(process.env.ISAT_BIN_DIR);

  // System packages (InsightAT .deb installs binaries here + /usr/bin symlinks).
  candidates.push('/usr/lib/insightat/bin');
  candidates.push('/usr/bin');
  if (process.env.PATH) {
    for (const dir of String(process.env.PATH).split(path.delimiter)) {
      if (dir) candidates.push(dir);
    }
  }

  // Next to the packaged GUI binary: .../linux-unpacked/bin or .../linux-unpacked/
  if (process.execPath) {
    const exeDir = path.dirname(process.execPath);
    candidates.push(path.join(exeDir, 'bin'));
    candidates.push(exeDir);
    // Common layout: GUI under dist/, CLI under repo build/
    candidates.push(path.resolve(exeDir, '..', '..', '..', 'build'));
    candidates.push(path.resolve(exeDir, '..', '..', 'build'));
  }

  // Dev: sfm-gui/src → repo root
  const repoRoot = path.resolve(__dirname, '..', '..');
  for (const dir of ['build', 'build-release', 'build-ceres-12.8', 'build-local']) {
    candidates.push(path.join(repoRoot, dir));
  }

  for (const dir of candidates) {
    if (hasCliBinary(dir)) return path.resolve(dir);
  }
  return '';
}

function ensureBinDir(state) {
  if (state && state.binDir && hasCliBinary(state.binDir)) return state;
  const resolved = resolveCliBinDir();
  if (!resolved) return state;
  return { ...state, binDir: resolved };
}

function commandCandidates(binDir, exeName) {
  const candidates = [];
  const autoBin = resolveCliBinDir();
  if (autoBin) candidates.push(path.join(autoBin, exeName));
  if (binDir) candidates.push(path.join(binDir, exeName));
  if (process.env.ISAT_BIN_DIR) candidates.push(path.join(process.env.ISAT_BIN_DIR, exeName));

  const repoRoot = path.resolve(__dirname, '..', '..');
  for (const dir of ['build', 'build-release', 'build-ceres-12.8', 'build-local']) {
    candidates.push(path.join(repoRoot, dir, exeName));
  }
  candidates.push(exeName);
  return candidates;
}

function findSfmViewerApp() {
  const candidates = [];
  const settings = loadUserSettings();
  if (settings.sfmViewerPath) candidates.push(settings.sfmViewerPath);

  // Bundled inside the GUI package (electron-builder extraResources).
  if (process.resourcesPath) {
    candidates.push(path.join(process.resourcesPath, 'sfm-viewer'));
  }

  // Standalone / unified install paths (no spaces in /opt/insightat*).
  candidates.push('/usr/bin/insightat-sfm-viewer');
  candidates.push('/opt/insightat/insightat-sfm-viewer');
  candidates.push('/opt/insightat-viewer/insightat-sfm-viewer');
  if (process.execPath) {
    candidates.push(path.join(path.dirname(process.execPath), 'insightat-sfm-viewer'));
  }
  if (process.env.PATH) {
    for (const dir of String(process.env.PATH).split(path.delimiter)) {
      if (dir) candidates.push(path.join(dir, 'insightat-sfm-viewer'));
    }
  }

  // Dev: sfm-gui/src → repo/sfm-viewer
  candidates.push(path.resolve(__dirname, '..', '..', 'sfm-viewer'));

  for (const dir of candidates) {
    if (!dir) continue;
    const resolved = path.resolve(dir);
    if (isSfmViewerLaunchable(resolved)) return resolved;
  }
  return '';
}

function findTool(binDir, exeName) {
  for (const candidate of commandCandidates(binDir, exeName)) {
    if (candidate === exeName) return candidate;
    if (fs.existsSync(candidate)) return candidate;
  }
  return exeName;
}

function sensorDbCandidates(binDir) {
  const candidates = [];
  if (binDir) {
    candidates.push(path.join(binDir, 'data', 'config', 'camera_sensor_database.txt'));
    candidates.push(path.join(binDir, 'config', 'camera_sensor_database.txt'));
  }
  if (process.env.ISAT_BIN_DIR) {
    candidates.push(path.join(process.env.ISAT_BIN_DIR, 'data', 'config', 'camera_sensor_database.txt'));
    candidates.push(path.join(process.env.ISAT_BIN_DIR, 'config', 'camera_sensor_database.txt'));
  }
  // System .deb layout
  candidates.push('/usr/share/insightat/data/config/camera_sensor_database.txt');
  candidates.push('/usr/lib/insightat/bin/data/config/camera_sensor_database.txt');

  const repoRoot = path.resolve(__dirname, '..', '..');
  for (const dir of ['build', 'build-release', 'build-ceres-12.8', 'build-local']) {
    candidates.push(path.join(repoRoot, dir, 'data', 'config', 'camera_sensor_database.txt'));
    candidates.push(path.join(repoRoot, dir, 'config', 'camera_sensor_database.txt'));
  }
  return candidates;
}

function findSensorDb(binDir) {
  return sensorDbCandidates(binDir).find((candidate) => fs.existsSync(candidate)) || '';
}

function parseEvents(text) {
  const events = [];
  for (const line of String(text).split(/\r?\n/)) {
    const prefix = 'ISAT_EVENT ';
    const idx = line.indexOf(prefix);
    if (idx < 0) continue;
    try {
      events.push(JSON.parse(line.slice(idx + prefix.length)));
    } catch (_) {
      // Keep raw logs usable even when a malformed line appears.
    }
  }
  return events;
}

/** Set when Stop is pressed; cleared by resetAbort() at the start of a job. */
let abortRequested = false;
/** Currently tracked CLI child (direct spawn of isat_*). */
let activeChild = null;
/** Nested/async UI jobs that must block project switches (reconstruction, etc.). */
let activeJobs = 0;

function beginJob() {
  activeJobs += 1;
}

function endJob() {
  activeJobs = Math.max(0, activeJobs - 1);
}

function resetAbort() {
  abortRequested = false;
}

function cancelledError(message = 'Stopped by user') {
  const err = new Error(message);
  err.cancelled = true;
  return err;
}

function assertNotAborted() {
  if (abortRequested) throw cancelledError();
}

/**
 * Kill a process and its descendants.
 * Windows: taskkill /T /F
 * Unix: SIGTERM then SIGKILL on the process group (spawned detached → new PGID).
 */
function killProcessTree(pid, hard = false) {
  if (!pid || pid <= 0) return;
  if (process.platform === 'win32') {
    spawnSync('taskkill', ['/pid', String(pid), '/T', '/F'], {
      windowsHide: true,
      stdio: 'ignore'
    });
    return;
  }
  const signal = hard ? 'SIGKILL' : 'SIGTERM';
  try {
    process.kill(-pid, signal);
  } catch (_) {
    try {
      process.kill(pid, signal);
    } catch (__) {
      /* already gone */
    }
  }
  if (hard) {
    for (const childPid of listDescendantPids(pid)) {
      try {
        process.kill(childPid, 'SIGKILL');
      } catch (_) {
        /* ignore */
      }
    }
  }
}

function listDescendantPids(rootPid) {
  const out = [];
  try {
    const result = spawnSync('ps', ['-o', 'pid=,ppid=', '-ax'], {
      encoding: 'utf8',
      timeout: 3000
    });
    if (result.status !== 0 || !result.stdout) return out;
    const children = new Map();
    for (const line of result.stdout.split('\n')) {
      const parts = line.trim().split(/\s+/);
      if (parts.length < 2) continue;
      const pid = Number(parts[0]);
      const ppid = Number(parts[1]);
      if (!Number.isInteger(pid) || !Number.isInteger(ppid)) continue;
      if (!children.has(ppid)) children.set(ppid, []);
      children.get(ppid).push(pid);
    }
    const stack = [rootPid];
    const seen = new Set();
    while (stack.length) {
      const cur = stack.pop();
      for (const c of children.get(cur) || []) {
        if (seen.has(c)) continue;
        seen.add(c);
        out.push(c);
        stack.push(c);
      }
    }
  } catch (_) {
    /* ignore */
  }
  return out;
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

/**
 * Request stop of the active CLI tree. Safe to call when idle.
 * @returns {{ stopped: boolean, hadProcess: boolean }}
 */
async function stopActive(onLog) {
  abortRequested = true;
  stopLogTailer();
  const child = activeChild;
  if (!child || !child.pid) {
    if (onLog) onLog('# Stop requested\n');
    return { stopped: true, hadProcess: false };
  }
  const pid = child.pid;
  if (onLog) onLog(`# Stopping process tree (pid ${pid})…\n`);
  killProcessTree(pid, false);
  const deadline = Date.now() + 2500;
  while (activeChild === child && Date.now() < deadline) {
    await sleep(100);
  }
  if (activeChild === child) {
    if (onLog) onLog('# Force-killing remaining processes…\n');
    killProcessTree(pid, true);
    await sleep(200);
  }
  if (activeChild === child) {
    activeChild = null;
  }
  if (onLog) onLog('# Stopped\n');
  return { stopped: true, hadProcess: true };
}

function isBusy() {
  return activeJobs > 0 || Boolean(activeChild);
}

function hasActiveChild() {
  return Boolean(activeChild && activeChild.pid);
}

let activeLogTailer = null;

function stopLogTailer() {
  if (activeLogTailer) {
    activeLogTailer.stop();
    activeLogTailer = null;
  }
}

/**
 * Tail work/logs/run_* for console/detail/progress. Returns a stop function.
 * Follows logs/current.json and switches when a new run_* appears.
 */
function startLogTailer(workDir, { onConsole, onDetail, onProgress, onRunSwitch, waitForNewRun } = {}) {
  stopLogTailer();
  const existing = readCurrentLogDir(workDir);
  const tailer = createLogTailer({
    onConsole,
    onDetail,
    onProgress,
    onRunSwitch
  });
  activeLogTailer = tailer;
  tailer.start(workDir, {
    baselineRunId: (existing && existing.runId) || '',
    waitForNewRun: waitForNewRun != null ? waitForNewRun : Boolean(existing && existing.runId)
  });
  return () => {
    if (activeLogTailer === tailer) {
      tailer.stop();
      activeLogTailer = null;
    }
  };
}

const PIPELINE_STATUS_FILE = 'insightat-pipeline-status.json';

/** User-facing stages and dependency graph. */
const PIPELINE_STAGES = [
  {
    id: 'features',
    label: 'Features',
    dependsOn: [],
    cliSteps: ['extract'],
    dirs: ['feat', 'feat_retrieval'],
    files: []
  },
  {
    id: 'matching',
    label: 'Matching',
    dependsOn: ['features'],
    cliSteps: ['match'],
    dirs: ['match', 'geo', 'retrieval_match_work'],
    files: ['camera_estimate_meta.json']
  },
  {
    id: 'sfm',
    label: 'SfM',
    dependsOn: ['matching'],
    cliSteps: ['tracks', 'seed_eval', 'incremental_sfm', 'undistort'],
    dirs: ['incremental_sfm', 'seed_eval_all', 'tracks'],
    files: []
  }
];

const STAGE_BY_ID = Object.fromEntries(PIPELINE_STAGES.map((s) => [s.id, s]));

function pipelineStatusPath(workDir) {
  return path.join(workDir, PIPELINE_STATUS_FILE);
}

function defaultPipelineStatus() {
  const stages = {};
  for (const stage of PIPELINE_STAGES) {
    stages[stage.id] = { status: 'pending', updatedAt: null };
  }
  return { version: 1, stages };
}

/** True only when stage outputs look finished (not merely that a dir exists). */
function inferStageComplete(workDir, stageId) {
  if (stageId === 'features') {
    const featDir = path.join(workDir, 'feat');
    if (!fs.existsSync(featDir)) return false;
    try {
      return fs.readdirSync(featDir).some((name) => name.endsWith('.isat_feat'));
    } catch (_) {
      return false;
    }
  }
  if (stageId === 'matching') {
    return fs.existsSync(path.join(workDir, 'geo', 'pairs.json'));
  }
  if (stageId === 'sfm') {
    return (
      fs.existsSync(path.join(workDir, 'incremental_sfm', 'poses.json')) ||
      Boolean(reconstructionViewPath(workDir))
    );
  }
  return false;
}

/** @deprecated use inferStageComplete */
function inferStageDone(workDir, stageId) {
  return inferStageComplete(workDir, stageId);
}

/**
 * Status file is authoritative.
 * - Honor pending / failed / running as written.
 * - Demote done → pending only when completion artifacts are missing.
 * - Bootstrap from disk only when a stage has never been recorded.
 */
function loadPipelineStatus(workDir) {
  let stored = null;
  try {
    const file = pipelineStatusPath(workDir);
    if (fs.existsSync(file)) {
      stored = JSON.parse(fs.readFileSync(file, 'utf8'));
    }
  } catch (_) {
    stored = null;
  }

  const merged = defaultPipelineStatus();
  const honorRunning = isBusy();
  for (const stage of PIPELINE_STAGES) {
    const fromFile = stored && stored.stages && stored.stages[stage.id];
    const recorded = fromFile && fromFile.status ? fromFile.status : null;
    let status;

    if (recorded === 'running') {
      status = honorRunning ? 'running' : 'pending';
    } else if (recorded === 'pending' || recorded === 'failed' || recorded === 'done') {
      status = recorded;
      if (status === 'done' && !inferStageComplete(workDir, stage.id)) {
        status = 'pending';
      }
    } else {
      // No marker yet (legacy workdir): bootstrap once from artifacts.
      status = inferStageComplete(workDir, stage.id) ? 'done' : 'pending';
    }

    merged.stages[stage.id] = {
      status,
      updatedAt: (fromFile && fromFile.updatedAt) || null
    };
  }

  for (const stage of PIPELINE_STAGES) {
    for (const dep of stage.dependsOn) {
      if (merged.stages[dep].status !== 'done' && merged.stages[stage.id].status === 'done') {
        merged.stages[stage.id] = { status: 'pending', updatedAt: null };
      }
    }
  }

  return merged;
}

function savePipelineStatus(workDir, status) {
  fs.mkdirSync(workDir, { recursive: true });
  fs.writeFileSync(pipelineStatusPath(workDir), `${JSON.stringify(status, null, 2)}\n`);
  return status;
}

function setStageStatus(workDir, stageId, status) {
  // Read raw file — avoid loadPipelineStatus() flipping running→done via artifact inference.
  let next = defaultPipelineStatus();
  try {
    const file = pipelineStatusPath(workDir);
    if (fs.existsSync(file)) {
      const stored = JSON.parse(fs.readFileSync(file, 'utf8'));
      if (stored && stored.stages) {
        for (const stage of PIPELINE_STAGES) {
          if (stored.stages[stage.id]) next.stages[stage.id] = stored.stages[stage.id];
        }
      }
    }
  } catch (_) {
    /* keep defaults */
  }
  next.stages[stageId] = {
    status,
    updatedAt: new Date().toISOString()
  };
  return savePipelineStatus(workDir, next);
}

function cleanStageArtifacts(workDir, stageId, onLog) {
  const stage = STAGE_BY_ID[stageId];
  if (!stage) return;
  for (const name of stage.dirs) {
    const target = path.join(workDir, name);
    if (!fs.existsSync(target)) continue;
    fs.rmSync(target, { recursive: true, force: true });
    if (onLog) onLog(`# Removed ${name}/\n`);
  }
  for (const name of stage.files) {
    const target = path.join(workDir, name);
    if (!fs.existsSync(target)) continue;
    fs.unlinkSync(target);
    if (onLog) onLog(`# Removed ${name}\n`);
  }
}

function cleanFromStage(workDir, fromStageId, onLog) {
  const start = PIPELINE_STAGES.findIndex((s) => s.id === fromStageId);
  if (start < 0) return;
  for (let i = start; i < PIPELINE_STAGES.length; i++) {
    cleanStageArtifacts(workDir, PIPELINE_STAGES[i].id, onLog);
  }
}

function getPipelinePlan(stateOrWorkDir) {
  const workDir = typeof stateOrWorkDir === 'string'
    ? stateOrWorkDir
    : stateOrWorkDir.workDir;
  const status = loadPipelineStatus(workDir);
  const stages = PIPELINE_STAGES.map((stage) => ({
    id: stage.id,
    label: stage.label,
    dependsOn: stage.dependsOn.slice(),
    status: status.stages[stage.id].status
  }));

  let resumeFrom = null;
  for (const stage of stages) {
    if (stage.status !== 'done') {
      resumeFrom = stage.id;
      break;
    }
  }

  const allDone = resumeFrom === null;
  const anyStarted = stages.some((s) => s.status === 'done' || s.status === 'failed');
  const needsChoice = anyStarted;
  let defaultMode = 'force';
  if (!anyStarted) defaultMode = 'force';
  else if (!allDone) defaultMode = 'continue';
  else defaultMode = 'force';

  return {
    stages,
    resumeFrom: resumeFrom || 'features',
    allDone,
    anyStarted,
    needsChoice,
    defaultMode
  };
}

function cliStepsFrom(stageId) {
  const start = PIPELINE_STAGES.findIndex((s) => s.id === stageId);
  if (start < 0) return PIPELINE_STAGES.flatMap((s) => s.cliSteps);
  const steps = [];
  for (let i = start; i < PIPELINE_STAGES.length; i++) {
    steps.push(...PIPELINE_STAGES[i].cliSteps);
  }
  return steps.filter((s, i, arr) => arr.indexOf(s) === i);
}

function runCommand(state, exeName, args, onLog, options = {}) {
  return new Promise((resolve, reject) => {
    try {
      assertNotAborted();
    } catch (err) {
      reject(err);
      return;
    }

    const command = findTool(state.binDir, exeName);
    const mirrorPipe = options.mirrorPipe !== false;
    // Unix: new process group so we can kill(-pid) the whole CLI tree.
    // Windows: taskkill /T walks the tree; detached not required.
    const child = spawn(command, args, {
      cwd: state.workDir,
      env: buildEnv(state, command),
      detached: process.platform !== 'win32',
      windowsHide: true
    });
    activeChild = child;
    let output = '';
    let settled = false;

    const send = (chunk) => {
      const text = chunk.toString();
      output += text;
      if (mirrorPipe && onLog) onLog(text);
    };

    const finish = (fn) => {
      if (settled) return;
      settled = true;
      if (activeChild === child) activeChild = null;
      fn();
    };

    child.stdout.on('data', send);
    child.stderr.on('data', send);
    child.on('error', (err) => {
      finish(() => reject(new Error(`Failed to start ${exeName}: ${err.message}`)));
    });
    child.on('close', (code, signal) => {
      finish(() => {
        const events = parseEvents(output);
        if (abortRequested || signal === 'SIGTERM' || signal === 'SIGKILL') {
          reject(cancelledError());
          return;
        }
        if (code !== 0) {
          const err = new Error(`${exeName} exited with code ${code}`);
          err.output = output;
          err.events = events;
          reject(err);
          return;
        }
        resolve({ command, args, output, events });
      });
    });
  });
}

function buildEnv(state, command) {
  const env = { ...process.env };
  const binDir = path.dirname(command);
  const extraLibs = [
    binDir,
    path.join(binDir, 'third_party', 'popsift', 'Linux-x86_64')
  ];
  env.LD_LIBRARY_PATH = [extraLibs.join(':'), env.LD_LIBRARY_PATH || ''].filter(Boolean).join(':');
  return env;
}

function lastEventData(result, type) {
  for (let i = result.events.length - 1; i >= 0; --i) {
    const event = result.events[i];
    if (!type || event.type === type) return event.data || {};
  }
  return {};
}

async function createProject(options, onLog) {
  resetAbort();
  const opts = { ...options };
  if (!opts.binDir) opts.binDir = resolveCliBinDir();
  const state = defaultState(options.workDir, opts);
  fs.mkdirSync(state.workDir, { recursive: true });
  await runCommand(state, 'isat_project', ['create', '-p', state.projectPath, '-n', state.name], onLog);
  const saved = saveState(state);
  touchRecentProject(saved);
  return saved;
}

async function openProject(workDir) {
  const state = loadState(workDir);
  touchRecentProject(state);
  return state;
}

async function addFolder(state, folderPath, options = {}, onLog) {
  resetAbort();
  let next = ensureBinDir({ ...state });
  if (options.binDir) next.binDir = options.binDir;
  if (Object.prototype.hasOwnProperty.call(options, 'ext')) next.ext = normalizeExts(options.ext);
  if (Object.prototype.hasOwnProperty.call(options, 'maxSample')) {
    const maxSample = Number.parseInt(options.maxSample, 10);
    if (Number.isInteger(maxSample) && maxSample > 0) next.maxSample = maxSample;
  }
  next.ext = normalizeExts(next.ext);
  const folder = path.resolve(folderPath);
  if (!fs.existsSync(next.projectPath)) {
    throw new Error(`Project file does not exist: ${next.projectPath}`);
  }
  if (!fs.statSync(folder).isDirectory()) {
    throw new Error(`Image folder does not exist: ${folder}`);
  }

  const groups = scanGroups(folder, next.ext);
  if (groups.length === 0) {
    throw new Error(`No images found under ${folder}`);
  }

  const addedGroupIds = [];
  for (const group of groups) {
    const addGroup = await runCommand(
      next,
      'isat_project',
      ['add-group', '-p', next.projectPath, '-n', uniqueGroupName(next, group.name)],
      onLog
    );
    const data = lastEventData(addGroup, 'project.add_group');
    const groupId = data.group_id;
    if (!Number.isInteger(groupId)) {
      throw new Error(`Could not read group_id for ${group.name}`);
    }

    await runCommand(
      next,
      'isat_project',
      ['add-images', '-p', next.projectPath, '-g', String(groupId), '-i', group.path, '--ext', next.ext],
      onLog
    );

    next.groups.push({
      groupId,
      name: data.group_name || group.name,
      folder: group.path,
      intrinsicsSource: next.cameraMode === 'manual' ? 'manual' : 'auto',
      fixIntrinsics: false,
      width: 0,
      height: 0
    });
    addedGroupIds.push(groupId);
  }

  const cameraArgs = ['-p', next.projectPath, '-a', '--max-sample', String(next.maxSample), '--auto-split'];
  const sensorDb = findSensorDb(next.binDir);
  if (sensorDb) cameraArgs.push('-d', sensorDb);
  await runCommand(next, 'isat_camera_estimator', cameraArgs, onLog);

  if (!next.folders.includes(folder)) next.folders.push(folder);

  // Project is Manual: give each new group its own seeded camera (not shared).
  if (next.cameraMode === 'manual') {
    const newIds = new Set(addedGroupIds);
    for (const g of next.groups || []) {
      if (!newIds.has(g.groupId)) continue;
      next = await applyCameraToOneGroup(next, g.groupId, defaultCameraForGroup(g), onLog);
    }
  } else {
    next.cameraMode = 'auto';
  }

  return saveState(next);
}

function uniqueGroupName(state, baseName) {
  const existing = new Set((state.groups || []).map((group) => group.name));
  if (!existing.has(baseName)) return baseName;
  let i = 2;
  while (existing.has(`${baseName}_${i}`)) i += 1;
  return `${baseName}_${i}`;
}

async function prepareImagesAll(state, options = {}, onLog) {
  let next = ensureBinDir({ ...state });
  if (options.binDir) next.binDir = options.binDir;
  if (!next.groups || next.groups.length === 0) {
    throw new Error('Add at least one image folder before reconstruction.');
  }
  const taskName = `SfM_${new Date().toISOString().replace(/[-:.TZ]/g, '').slice(0, 14)}`;
  const createTask = await runCommand(
    next,
    'isat_project',
    ['create-at-task', '-p', next.projectPath, '-n', taskName],
    onLog
  );
  const data = lastEventData(createTask, 'project.create_at_task');
  const taskId = data.task_id;
  if (!Number.isInteger(taskId)) {
    throw new Error('Could not read task_id from create-at-task.');
  }
  next.latestTaskId = taskId;
  await runCommand(
    next,
    'isat_project',
    ['extract', '-p', next.projectPath, '-t', String(taskId), '-o', next.imagesAllPath, '-a'],
    onLog
  );
  return saveState(next);
}

async function runReconstruction(state, options = {}, onLog) {
  resetAbort();
  beginJob();
  try {
    return await runReconstructionInner(state, options, onLog);
  } finally {
    endJob();
  }
}

async function runReconstructionInner(state, options = {}, onLog) {
  const plan = getPipelinePlan(state);
  let mode = options.mode;

  if (plan.needsChoice && mode !== 'continue' && mode !== 'force') {
    const err = new Error('Choose Continue or Rebuild to start reconstruction.');
    err.needsChoice = true;
    err.plan = plan;
    throw err;
  }
  if (!mode) mode = plan.defaultMode;

  const fromStage = mode === 'force' ? 'features' : plan.resumeFrom;
  if (mode === 'force') {
    if (onLog) onLog('# Rebuild: cleaning pipeline outputs from Features\n');
    cleanFromStage(state.workDir, 'features', onLog);
    const imagesAll = path.join(state.workDir, 'images_all.json');
    if (fs.existsSync(imagesAll)) {
      fs.unlinkSync(imagesAll);
      if (onLog) onLog('# Removed images_all.json\n');
    }
  } else if (fromStage) {
    if (onLog) onLog(`# Continue from ${fromStage}\n`);
    cleanFromStage(state.workDir, fromStage, onLog);
  }

  {
    const st = loadPipelineStatus(state.workDir);
    const startIdx = PIPELINE_STAGES.findIndex((s) => s.id === fromStage);
    for (let i = Math.max(0, startIdx); i < PIPELINE_STAGES.length; i++) {
      st.stages[PIPELINE_STAGES[i].id] = { status: 'pending', updatedAt: null };
    }
    savePipelineStatus(state.workDir, st);
  }

  const next = await prepareImagesAll(state, options, onLog);
  assertNotAborted();

  const steps = cliStepsFrom(fromStage);
  const args = [
    '--existing-task',
    '-w', next.workDir,
    '-v',
    '--undistort',
    '--binary',
    '--steps', steps.join(',')
  ];

  const backend = detectComputeBackend();
  if (backend.mode === 'glsl') {
    args.push('--extract-backend', 'glsl', '--match-backend', 'glsl');
    if (onLog) onLog('# Compute backend: GLSL (CUDA not detected)\n');
  } else if (onLog) {
    onLog('# Compute backend: CUDA\n');
  }

  const manualFix =
    next.cameraMode === 'manual' &&
    (next.groups || []).some((g) => g.fixIntrinsics);
  if (manualFix) {
    args.push('--fix-intrinsics');
    if (onLog) onLog('# Fixing intrinsics for manually set cameras\n');
  }

  if (onLog) onLog(`# Pipeline steps: ${steps.join(' → ')}\n`);
  if (onLog) onLog('# Tailing work/logs/run_* (follows logs/current.json)\n');

  const startIdx = PIPELINE_STAGES.findIndex((s) => s.id === fromStage);
  for (let i = Math.max(0, startIdx); i < PIPELINE_STAGES.length; i++) {
    setStageStatus(next.workDir, PIPELINE_STAGES[i].id, 'running');
  }
  if (typeof options.onPlan === 'function') {
    options.onPlan(getPipelinePlan(next.workDir));
  }
  if (typeof options.onLogReset === 'function') {
    options.onLogReset({ reason: 'run-start' });
  }
  if (typeof options.onProgress === 'function') {
    options.onProgress({
      overall: 0,
      fraction: 0,
      message: 'Starting reconstruction',
      done: false
    });
  }

  const stopTail = startLogTailer(next.workDir, {
    onConsole: (text) => {
      if (typeof options.onConsole === 'function') options.onConsole(text);
      else if (onLog) onLog(text);
    },
    onDetail: (text) => {
      if (typeof options.onDetail === 'function') options.onDetail(text);
    },
    onProgress: typeof options.onProgress === 'function' ? options.onProgress : undefined,
    onRunSwitch: (info) => {
      if (typeof options.onConsole === 'function') {
        options.onConsole(`# Switched to log ${info.runId || path.basename(info.dir || '')}\n`);
      } else if (onLog) {
        onLog(`# Switched to log ${info.runId || path.basename(info.dir || '')}\n`);
      }
      if (typeof options.onRunDir === 'function') {
        options.onRunDir(info);
      }
    },
    waitForNewRun: true
  });

  try {
    await runCommand(next, 'isat_sfm', args, onLog, { mirrorPipe: false });
    for (let i = Math.max(0, startIdx); i < PIPELINE_STAGES.length; i++) {
      setStageStatus(next.workDir, PIPELINE_STAGES[i].id, 'done');
    }
  } catch (err) {
    const cancelled = Boolean(err && err.cancelled);
    for (let i = Math.max(0, startIdx); i < PIPELINE_STAGES.length; i++) {
      const id = PIPELINE_STAGES[i].id;
      // Status markers are authoritative after this write. Only mark done when this run
      // actually produced completion artifacts (Rebuild already cleaned old ones).
      if (inferStageComplete(next.workDir, id)) {
        setStageStatus(next.workDir, id, 'done');
      } else {
        setStageStatus(next.workDir, id, cancelled ? 'pending' : 'failed');
      }
    }
    if (typeof options.onPlan === 'function') {
      options.onPlan(getPipelinePlan(next.workDir));
    }
    // Still return a summary so the UI can refresh chips after Stop.
    err.summary = loadSummary(saveState(next));
    throw err;
  } finally {
    stopTail();
  }

  if (typeof options.onProgress === 'function') {
    options.onProgress({
      overall: 1,
      fraction: 1,
      message: 'Pipeline complete',
      done: true
    });
  }
  if (typeof options.onPlan === 'function') {
    options.onPlan(getPipelinePlan(next.workDir));
  }

  const summary = loadSummary(saveState(next));
  touchRecentProject(summary);
  return summary;
}

function detectComputeBackend() {
  try {
    const result = spawnSync('nvidia-smi', ['-L'], {
      encoding: 'utf8',
      timeout: 3000,
      stdio: ['ignore', 'pipe', 'ignore']
    });
    if (result.status === 0 && String(result.stdout || '').trim()) {
      return { mode: 'cuda', label: 'CUDA' };
    }
  } catch (_) {
    /* ignore */
  }
  return { mode: 'glsl', label: 'Compatible (GLSL)' };
}

function buildSetCameraArgs(projectPath, groupId, camera) {
  const fx = Number(camera.fx);
  if (!(fx > 0)) throw new Error('fx must be a positive pixel focal length');

  const convention = camera.brownConvention === 'opencv' ? 'opencv' : 'context-capture';
  const args = [
    'set-camera',
    '-p', projectPath,
    '-g', String(groupId),
    '--fx', String(fx),
    '--brown-convention', convention
  ];

  const maybe = (key, flag) => {
    if (camera[key] === undefined || camera[key] === null || camera[key] === '') return;
    const n = Number(camera[key]);
    if (Number.isFinite(n)) args.push(flag, String(n));
  };
  maybe('fy', '--fy');
  maybe('cx', '--cx');
  maybe('cy', '--cy');
  maybe('k1', '--k1');
  maybe('k2', '--k2');
  maybe('k3', '--k3');
  maybe('p1', '--p1');
  maybe('p2', '--p2');
  return { args, fx, convention };
}

function defaultCameraForGroup(group) {
  const w = Number(group.width) > 0 ? Number(group.width) : 4000;
  const h = Number(group.height) > 0 ? Number(group.height) : Math.round(w * 0.75);
  const fx = Number(group.fx) > 0 ? Number(group.fx) : Math.round(w * 0.9);
  const fy = Number(group.fy) > 0 ? Number(group.fy) : fx;
  return {
    fx,
    fy,
    cx: group.cx !== undefined && group.cx !== null && group.cx !== '' ? Number(group.cx) : w / 2,
    cy: group.cy !== undefined && group.cy !== null && group.cy !== '' ? Number(group.cy) : h / 2,
    k1: Number(group.k1) || 0,
    k2: Number(group.k2) || 0,
    k3: Number(group.k3) || 0,
    p1: Number(group.p1) || 0,
    p2: Number(group.p2) || 0,
    brownConvention: group.brownConvention === 'opencv' ? 'opencv' : 'context-capture',
    fixIntrinsics: Boolean(group.fixIntrinsics)
  };
}

async function applyCameraToOneGroup(state, groupId, camera, onLog) {
  let next = ensureBinDir({ ...state });
  const gid = Number(groupId);
  if (!Number.isInteger(gid)) throw new Error('Invalid group id');
  const group = (next.groups || []).find((g) => g.groupId === gid);
  if (!group) throw new Error(`Group ${gid} not found in project state`);

  const { args, fx, convention } = buildSetCameraArgs(next.projectPath, gid, camera);
  const result = await runCommand(next, 'isat_project', args, onLog);
  const data = lastEventData(result, 'project.set_camera');
  const snap = {
    fx: data.fx || fx,
    fy: data.fy || fx,
    cx: data.cx,
    cy: data.cy,
    k1: Number(camera.k1) || 0,
    k2: Number(camera.k2) || 0,
    k3: Number(camera.k3) || 0,
    p1: Number(camera.p1) || 0,
    p2: Number(camera.p2) || 0,
    brownConvention: convention,
    fixIntrinsics: Boolean(camera.fixIntrinsics)
  };

  next.groups = (next.groups || []).map((g) => {
    if (g.groupId !== gid) return g;
    return {
      ...g,
      intrinsicsSource: 'manual',
      fixIntrinsics: snap.fixIntrinsics,
      width: data.width || g.width || 0,
      height: data.height || g.height || 0,
      ...snap
    };
  });
  next.cameraMode = 'manual';
  return next;
}

/** Set camera for one group. Project mode becomes Manual. */
async function setGroupCamera(state, groupId, camera, onLog) {
  resetAbort();
  const next = await applyCameraToOneGroup(state, groupId, camera, onLog);
  return loadSummary(saveState(next));
}

/**
 * Switch project to Manual and seed every group with a usable default camera
 * (existing values kept when present).
 */
async function enterProjectCameraManual(state, onLog) {
  resetAbort();
  let next = ensureBinDir({ ...state });
  const groups = next.groups || [];
  if (groups.length === 0) throw new Error('Add image folders before setting a camera.');

  for (const group of groups) {
    assertNotAborted();
    next = await applyCameraToOneGroup(next, group.groupId, defaultCameraForGroup(group), onLog);
  }
  next.cameraMode = 'manual';
  next.manualCamera = null;
  return loadSummary(saveState(next));
}

/** Re-estimate all groups and switch project to Auto. */
async function setProjectCameraAuto(state, onLog) {
  resetAbort();
  let next = ensureBinDir({ ...state });
  if (!next.groups || next.groups.length === 0) {
    throw new Error('Add image folders before setting a camera.');
  }

  const cameraArgs = [
    '-p', next.projectPath,
    '-a',
    '--max-sample', String(next.maxSample || 5),
    '--auto-split'
  ];
  const sensorDb = findSensorDb(next.binDir);
  if (sensorDb) cameraArgs.push('-d', sensorDb);
  await runCommand(next, 'isat_camera_estimator', cameraArgs, onLog);

  next.cameraMode = 'auto';
  next.manualCamera = null;
  next.groups = (next.groups || []).map((g) => ({
    ...g,
    intrinsicsSource: 'auto',
    fixIntrinsics: false,
    fx: undefined,
    fy: undefined,
    cx: undefined,
    cy: undefined,
    k1: undefined,
    k2: undefined,
    k3: undefined,
    p1: undefined,
    p2: undefined,
    brownConvention: undefined
  }));

  return loadSummary(saveState(next));
}

function profilePath() {
  const base = userDataDir
    || path.join(os.homedir(), '.config', 'InsightAT', 'sfm-gui');
  return path.join(base, 'profile.json');
}

function defaultProfile() {
  return {
    version: 1,
    compute: { prefer: 'auto' },
    cameraPresets: [],
    recentProjects: []
  };
}

function loadProfile() {
  try {
    const file = profilePath();
    if (!fs.existsSync(file)) return defaultProfile();
    const raw = JSON.parse(fs.readFileSync(file, 'utf8'));
    return {
      ...defaultProfile(),
      ...raw,
      cameraPresets: Array.isArray(raw.cameraPresets) ? raw.cameraPresets : [],
      recentProjects: Array.isArray(raw.recentProjects) ? raw.recentProjects : []
    };
  } catch (_) {
    return defaultProfile();
  }
}

function saveProfile(partial = {}) {
  const next = {
    ...loadProfile(),
    ...partial
  };
  if (!Array.isArray(next.cameraPresets)) next.cameraPresets = [];
  if (!Array.isArray(next.recentProjects)) next.recentProjects = [];
  const file = profilePath();
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, `${JSON.stringify(next, null, 2)}\n`);
  return next;
}

function touchRecentProject(state) {
  if (!state || !state.workDir) return loadProfile();
  const profile = loadProfile();
  const entry = {
    workDir: state.workDir,
    name: state.name || path.basename(state.workDir),
    openedAt: new Date().toISOString()
  };
  const rest = profile.recentProjects.filter((p) => p.workDir !== entry.workDir);
  profile.recentProjects = [entry, ...rest].slice(0, 12);
  return saveProfile(profile);
}

function saveCameraPreset(preset) {
  const name = String(preset.name || '').trim();
  if (!name) throw new Error('Preset name required');
  const fx = Number(preset.fx);
  if (!(fx > 0)) throw new Error('Preset fx must be positive');
  const item = {
    id: preset.id || `preset_${Date.now()}`,
    name,
    fx,
    fy: preset.fy !== undefined && preset.fy !== '' ? Number(preset.fy) : undefined,
    cx: preset.cx !== undefined && preset.cx !== '' ? Number(preset.cx) : undefined,
    cy: preset.cy !== undefined && preset.cy !== '' ? Number(preset.cy) : undefined,
    k1: Number(preset.k1) || 0,
    k2: Number(preset.k2) || 0,
    k3: Number(preset.k3) || 0,
    p1: Number(preset.p1) || 0,
    p2: Number(preset.p2) || 0,
    brownConvention: preset.brownConvention === 'opencv' ? 'opencv' : 'context-capture'
  };
  const profile = loadProfile();
  const others = profile.cameraPresets.filter((p) => p.name !== name && p.id !== item.id);
  profile.cameraPresets = [item, ...others].slice(0, 32);
  return saveProfile(profile);
}

function deleteCameraPreset(id) {
  const profile = loadProfile();
  profile.cameraPresets = profile.cameraPresets.filter((p) => p.id !== id);
  return saveProfile(profile);
}

function getSettingsInfo() {
  const settings = loadUserSettings();
  const backend = detectComputeBackend();
  return {
    ...settings,
    settingsPath: settingsPath(),
    resolvedBinDir: resolveCliBinDir(),
    resolvedViewer: findSfmViewerApp(),
    computeBackend: backend.label,
    computeMode: backend.mode
  };
}

function reconstructionViewPath(workDir) {
  const sfmDir = path.join(workDir, 'incremental_sfm');
  if (!fs.existsSync(sfmDir)) return null;

  // Prefer COLMAP binary (faster load); fall back to text.
  const colmapDir = path.join(sfmDir, 'colmap', 'sparse', '0');
  if (fs.existsSync(path.join(colmapDir, 'cameras.bin')) &&
      fs.existsSync(path.join(colmapDir, 'images.bin')) &&
      fs.existsSync(path.join(colmapDir, 'points3D.bin'))) {
    return colmapDir;
  }
  if (fs.existsSync(path.join(colmapDir, 'cameras.txt')) &&
      fs.existsSync(path.join(colmapDir, 'images.txt')) &&
      fs.existsSync(path.join(colmapDir, 'points3D.txt'))) {
    return colmapDir;
  }

  // Bundler: incremental_sfm/bundler/
  const bundlerDir = path.join(sfmDir, 'bundler');
  if (fs.existsSync(path.join(bundlerDir, 'bundle.out')) &&
      fs.existsSync(path.join(bundlerDir, 'list.txt'))) {
    return bundlerDir;
  }

  return null;
}

function loadSummary(state) {
  let imageCount = 0;
  if (fs.existsSync(state.imagesAllPath)) {
    try {
      const imagesAll = JSON.parse(fs.readFileSync(state.imagesAllPath, 'utf8'));
      imageCount = Array.isArray(imagesAll.images) ? imagesAll.images.length : 0;
    } catch (_) {
      imageCount = 0;
    }
  }
  const viewPath = reconstructionViewPath(state.workDir);
  const settings = loadUserSettings();
  // Prefer global settings binDir, then project binDir, then auto-detect.
  let resolvedBin = '';
  if (settings.binDir && hasCliBinary(settings.binDir)) {
    resolvedBin = path.resolve(settings.binDir);
  } else if (state.binDir && hasCliBinary(state.binDir)) {
    resolvedBin = state.binDir;
  } else {
    resolvedBin = resolveCliBinDir();
  }
  const backend = detectComputeBackend();
  return {
    ...state,
    binDir: resolvedBin || state.binDir || '',
    cliBinDir: resolvedBin || '',
    cliFound: Boolean(resolvedBin),
    sfmViewerPath: findSfmViewerApp() || '',
    computeBackend: backend.label,
    imageCount,
    groupCount: Array.isArray(state.groups) ? state.groups.length : 0,
    hasImagesAll: fs.existsSync(state.imagesAllPath),
    hasResult: fs.existsSync(path.join(state.workDir, 'incremental_sfm')),
    reconstructionViewPath: viewPath,
    hasManualIntrinsics: state.cameraMode === 'manual',
    cameraMode: state.cameraMode === 'manual' ? 'manual' : 'auto',
    pipelinePlan: getPipelinePlan(state.workDir)
  };
}

module.exports = {
  STATE_FILE,
  PROJECT_FILE,
  DEFAULT_EXT,
  normalizeExts,
  scanGroups,
  statePath,
  defaultState,
  loadState,
  saveState,
  setUserDataDir,
  loadUserSettings,
  saveUserSettings,
  getSettingsInfo,
  findTool,
  findSfmViewerApp,
  isSfmViewerAppDir,
  isSfmViewerLaunchable,
  packagedBinDir,
  resolveCliBinDir,
  detectComputeBackend,
  createProject,
  openProject,
  addFolder,
  prepareImagesAll,
  runReconstruction,
  getPipelinePlan,
  loadPipelineStatus,
  PIPELINE_STAGES,
  stopActive,
  resetAbort,
  isBusy,
  hasActiveChild,
  startLogTailer,
  stopLogTailer,
  readCurrentLogDir,
  setProjectCameraAuto,
  enterProjectCameraManual,
  setGroupCamera,
  loadProfile,
  saveProfile,
  touchRecentProject,
  saveCameraPreset,
  deleteCameraPreset,
  loadSummary,
  reconstructionViewPath
};
