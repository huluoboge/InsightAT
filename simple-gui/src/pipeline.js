'use strict';

const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

const STATE_FILE = 'insightat-simple-project.json';
const PROJECT_FILE = 'project.iat';
const DEFAULT_EXT = '.jpg,.jpeg,.tif,.tiff,.png';

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
    binDir: overrides.binDir || '',
    ext: normalizeExts(overrides.ext),
    maxSample: Number.isInteger(overrides.maxSample) ? overrides.maxSample : 5,
    folders: [],
    groups: [],
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
  return state;
}

function saveState(state) {
  const next = { ...state, updatedAt: new Date().toISOString() };
  fs.mkdirSync(next.workDir, { recursive: true });
  fs.writeFileSync(statePath(next.workDir), `${JSON.stringify(next, null, 2)}\n`);
  return next;
}

function commandCandidates(binDir, exeName) {
  const candidates = [];
  if (binDir) candidates.push(path.join(binDir, exeName));
  if (process.env.ISAT_BIN_DIR) candidates.push(path.join(process.env.ISAT_BIN_DIR, exeName));

  const repoRoot = path.resolve(__dirname, '..', '..');
  for (const dir of ['build-ceres-12.8', 'build-local', 'build']) {
    candidates.push(path.join(repoRoot, dir, exeName));
  }
  candidates.push(exeName);
  return candidates;
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
  const repoRoot = path.resolve(__dirname, '..', '..');
  candidates.push(path.join(repoRoot, 'build-ceres-12.8', 'data', 'config', 'camera_sensor_database.txt'));
  candidates.push(path.join(repoRoot, 'build-ceres-12.8', 'config', 'camera_sensor_database.txt'));
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

function runCommand(state, exeName, args, onLog) {
  return new Promise((resolve, reject) => {
    const command = findTool(state.binDir, exeName);
    const child = spawn(command, args, {
      cwd: state.workDir,
      env: buildEnv(state, command)
    });
    let output = '';

    const send = (chunk) => {
      const text = chunk.toString();
      output += text;
      if (onLog) onLog(text);
    };

    child.stdout.on('data', send);
    child.stderr.on('data', send);
    child.on('error', (err) => reject(new Error(`Failed to start ${exeName}: ${err.message}`)));
    child.on('close', (code) => {
      const events = parseEvents(output);
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
  const state = defaultState(options.workDir, options);
  fs.mkdirSync(state.workDir, { recursive: true });
  await runCommand(state, 'isat_project', ['create', '-p', state.projectPath, '-n', state.name], onLog);
  return saveState(state);
}

async function openProject(workDir) {
  return loadState(workDir);
}

async function addFolder(state, folderPath, options = {}, onLog) {
  const next = { ...state };
  if (Object.prototype.hasOwnProperty.call(options, 'binDir')) next.binDir = options.binDir || '';
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
      folder: group.path
    });
  }

  const cameraArgs = ['-p', next.projectPath, '-a', '--max-sample', String(next.maxSample), '--auto-split'];
  const sensorDb = findSensorDb(next.binDir);
  if (sensorDb) cameraArgs.push('-d', sensorDb);
  await runCommand(next, 'isat_camera_estimator', cameraArgs, onLog);

  if (!next.folders.includes(folder)) next.folders.push(folder);
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
  const next = { ...state };
  if (Object.prototype.hasOwnProperty.call(options, 'binDir')) next.binDir = options.binDir || '';
  if (!next.groups || next.groups.length === 0) {
    throw new Error('Add at least one image folder before reconstruction.');
  }
  const taskName = `Simple_${new Date().toISOString().replace(/[-:.TZ]/g, '').slice(0, 14)}`;
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
  const next = await prepareImagesAll(state, options, onLog);
  await runCommand(next, 'isat_sfm', ['--existing-task', '-w', next.workDir, '-v', '--undistort'], onLog);
  return loadSummary(saveState(next));
}

function reconstructionViewPath(workDir) {
  const sfmDir = path.join(workDir, 'incremental_sfm');
  if (!fs.existsSync(sfmDir)) return null;

  // Prefer COLMAP text format (more accurate camera models)
  const colmapDir = path.join(sfmDir, 'colmap', 'sparse', '0');
  if (fs.existsSync(path.join(colmapDir, 'cameras.txt')) &&
      fs.existsSync(path.join(colmapDir, 'images.txt')) &&
      fs.existsSync(path.join(colmapDir, 'points3D.txt'))) {
    return colmapDir;
  }

  // Check COLMAP binary format
  if (fs.existsSync(path.join(colmapDir, 'cameras.bin')) &&
      fs.existsSync(path.join(colmapDir, 'images.bin')) &&
      fs.existsSync(path.join(colmapDir, 'points3D.bin'))) {
    return colmapDir;
  }

  // Fall back to Bundler format
  if (fs.existsSync(path.join(sfmDir, 'bundle.out')) &&
      fs.existsSync(path.join(sfmDir, 'list.txt'))) {
    return sfmDir;
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
  return {
    ...state,
    imageCount,
    groupCount: Array.isArray(state.groups) ? state.groups.length : 0,
    hasImagesAll: fs.existsSync(state.imagesAllPath),
    hasResult: fs.existsSync(path.join(state.workDir, 'incremental_sfm')),
    reconstructionViewPath: viewPath
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
  findTool,
  createProject,
  openProject,
  addFolder,
  prepareImagesAll,
  runReconstruction,
  loadSummary,
  reconstructionViewPath
};
