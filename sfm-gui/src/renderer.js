'use strict';

const $ = (id) => document.getElementById(id);

const ui = {
  projectName: $('projectName'),
  createBtn: $('createBtn'),
  openBtn: $('openBtn'),
  addFolderBtn: $('addFolderBtn'),
  runBtn: $('runBtn'),
  stopBtn: $('stopBtn'),
  stageStatus: $('stageStatus'),
  runChoiceModal: $('runChoiceModal'),
  runChoiceSummary: $('runChoiceSummary'),
  runChoiceStages: $('runChoiceStages'),
  runChoiceCloseBtn: $('runChoiceCloseBtn'),
  runContinueBtn: $('runContinueBtn'),
  runForceBtn: $('runForceBtn'),
  viewBtn: $('viewBtn'),
  revealBtn: $('revealBtn'),
  settingsBtn: $('settingsBtn'),
  clearLogBtn: $('clearLogBtn'),
  logOutput: $('logOutput'),
  logDetailOutput: $('logDetailOutput'),
  logTabConsole: $('logTabConsole'),
  logTabDetail: $('logTabDetail'),
  progressWrap: $('progressWrap'),
  progressFill: $('progressFill'),
  progressLabel: $('progressLabel'),
  projectTitle: $('projectTitle'),
  projectSubtitle: $('projectSubtitle'),
  workDir: $('workDir'),
  groupCount: $('groupCount'),
  imageCount: $('imageCount'),
  cliBinDir: $('cliBinDir'),
  sfmViewerPath: $('sfmViewerPath'),
  computeBackend: $('computeBackend'),
  folderCount: $('folderCount'),
  groupList: $('groupList'),
  recentList: $('recentList'),
  runState: $('runState'),
  settingsModal: $('settingsModal'),
  settingsBinDir: $('settingsBinDir'),
  settingsViewer: $('settingsViewer'),
  settingsResolved: $('settingsResolved'),
  settingsError: $('settingsError'),
  browseBinBtn: $('browseBinBtn'),
  browseViewerBtn: $('browseViewerBtn'),
  settingsSaveBtn: $('settingsSaveBtn'),
  settingsClearBtn: $('settingsClearBtn'),
  settingsCloseBtn: $('settingsCloseBtn'),
  cameraModal: $('cameraModal'),
  cameraTitle: $('cameraTitle'),
  cameraSourceLabel: $('cameraSourceLabel'),
  cameraError: $('cameraError'),
  cameraCloseBtn: $('cameraCloseBtn'),
  cameraBtn: $('cameraBtn'),
  cameraModeBadge: $('cameraModeBadge'),
  cameraModeAuto: $('cameraModeAuto'),
  cameraModeManual: $('cameraModeManual'),
  cameraManualChrome: $('cameraManualChrome'),
  cameraManualPanel: $('cameraManualPanel'),
  camGroupSelect: $('camGroupSelect'),
  camFx: $('camFx'),
  camFy: $('camFy'),
  camCx: $('camCx'),
  camCy: $('camCy'),
  camK1: $('camK1'),
  camK2: $('camK2'),
  camK3: $('camK3'),
  camP1: $('camP1'),
  camP2: $('camP2'),
  camConvention: $('camConvention'),
  camFix: $('camFix'),
  camPresetSelect: $('camPresetSelect'),
  camApplyPresetBtn: $('camApplyPresetBtn'),
  camPresetName: $('camPresetName'),
  camSavePresetBtn: $('camSavePresetBtn')
};

let busy = false;
let state = null;
let cameraUiMode = 'auto';
let cameraTargetGroupId = null;
let profileCache = null;
let activeLogTab = 'console';

function appendLog(text) {
  if (!ui.logOutput) return;
  ui.logOutput.textContent += text;
  if (activeLogTab === 'console') {
    ui.logOutput.scrollTop = ui.logOutput.scrollHeight;
  }
}

function appendDetailLog(text) {
  if (!ui.logDetailOutput) return;
  ui.logDetailOutput.textContent += text;
  if (activeLogTab === 'detail') {
    ui.logDetailOutput.scrollTop = ui.logDetailOutput.scrollHeight;
  }
}

function setLogTab(tab) {
  activeLogTab = tab === 'detail' ? 'detail' : 'console';
  if (ui.logTabConsole) ui.logTabConsole.classList.toggle('active', activeLogTab === 'console');
  if (ui.logTabDetail) ui.logTabDetail.classList.toggle('active', activeLogTab === 'detail');
  if (ui.logOutput) ui.logOutput.hidden = activeLogTab !== 'console';
  if (ui.logDetailOutput) ui.logDetailOutput.hidden = activeLogTab !== 'detail';
}

function clearLogs() {
  if (ui.logOutput) ui.logOutput.textContent = '';
  if (ui.logDetailOutput) ui.logDetailOutput.textContent = '';
}

function resetProgress() {
  if (ui.progressWrap) ui.progressWrap.hidden = false;
  if (ui.progressFill) ui.progressFill.style.width = '0%';
  if (ui.progressLabel) ui.progressLabel.textContent = '0%';
}

function applyProgress(data) {
  if (!data || !ui.progressWrap) return;
  ui.progressWrap.hidden = false;
  const overall = Math.max(0, Math.min(1, Number(data.overall != null ? data.overall : data.fraction) || 0));
  if (ui.progressFill) ui.progressFill.style.width = `${(overall * 100).toFixed(1)}%`;
  const parts = [];
  if (data.step) parts.push(String(data.step));
  if (data.message) parts.push(String(data.message));
  if (data.current != null && data.total != null) {
    parts.push(`${data.current}/${data.total}${data.unit ? ' ' + data.unit : ''}`);
  }
  parts.push(`${(overall * 100).toFixed(0)}%`);
  if (ui.progressLabel) ui.progressLabel.textContent = parts.join(' · ');
  if (data.done && ui.runState) ui.runState.textContent = 'Reconstruction complete';
}

function setBusy(nextBusy, label) {
  busy = nextBusy;
  document.body.classList.toggle('busy', busy);
  for (const button of [ui.createBtn, ui.openBtn, ui.addFolderBtn, ui.runBtn, ui.viewBtn, ui.revealBtn, ui.cameraBtn, ui.settingsBtn]) {
    if (button) button.disabled = busy;
  }
  if (ui.projectName) ui.projectName.disabled = busy;
  if (ui.stopBtn) ui.stopBtn.disabled = !busy;
  ui.recentList.querySelectorAll('.recent-item').forEach((btn) => {
    btn.disabled = busy;
  });
  applyState(state);
  if (label) ui.runState.textContent = label;
}

function shortPath(value) {
  if (!value) return 'Not selected';
  if (value.length <= 46) return value;
  return `...${value.slice(-43)}`;
}

function setPathDisplay(el, value, emptyText) {
  if (!el) return;
  if (value) {
    el.textContent = shortPath(value);
    el.title = value;
  } else {
    el.textContent = emptyText;
    el.title = '';
  }
}

function applyPathInfo(info) {
  if (!info) return;
  setPathDisplay(
    ui.cliBinDir,
    info.path || info.resolvedBinDir || '',
    'Not found (build CLI first)'
  );
  setPathDisplay(
    ui.sfmViewerPath,
    info.viewer || info.resolvedViewer || '',
    'Not found'
  );
  if (ui.computeBackend) {
    ui.computeBackend.textContent = `Compute: ${info.computeBackend || info.computeMode || '…'}`;
  }
}

function applyState(nextState) {
  state = nextState;
  const hasProject = Boolean(state && state.workDir);
  const groupCount = hasProject ? state.groupCount || 0 : 0;
  const imageCount = hasProject ? state.imageCount || 0 : 0;

  ui.projectTitle.textContent = hasProject ? state.name : 'No project open';
  ui.projectSubtitle.textContent = hasProject ? state.projectPath : 'Create a project, add image folders, then run reconstruction.';
  ui.workDir.textContent = hasProject ? shortPath(state.workDir) : 'Not selected';
  ui.workDir.title = hasProject ? state.workDir : '';
  ui.groupCount.textContent = String(groupCount);
  ui.imageCount.textContent = String(imageCount);
  ui.folderCount.textContent = `${hasProject ? state.folders.length : 0} folders`;

  const cliPath = (state && (state.cliBinDir || state.binDir)) || '';
  if (cliPath) {
    setPathDisplay(ui.cliBinDir, cliPath, 'Not found (build CLI first)');
  } else if (state && state.cliFound === false) {
    setPathDisplay(ui.cliBinDir, '', 'Not found (build CLI first)');
  }
  if (state && state.sfmViewerPath !== undefined) {
    setPathDisplay(ui.sfmViewerPath, state.sfmViewerPath, 'Not found');
  }
  if (state && state.computeBackend) {
    ui.computeBackend.textContent = `Compute: ${state.computeBackend}`;
  }

  ui.addFolderBtn.disabled = busy || !hasProject;
  ui.runBtn.disabled = busy || !hasProject || groupCount === 0;
  if (ui.cameraBtn) ui.cameraBtn.disabled = busy || !hasProject || groupCount === 0;
  if (ui.stopBtn) ui.stopBtn.disabled = !busy;
  ui.revealBtn.disabled = busy || !hasProject;

  renderStageStatus(hasProject ? state.pipelinePlan : null);

  const cameraMode = hasProject && state.cameraMode === 'manual' ? 'manual' : 'auto';
  if (ui.cameraModeBadge) {
    if (hasProject && groupCount > 0) {
      ui.cameraModeBadge.hidden = false;
      ui.cameraModeBadge.className = `badge ${cameraMode}`;
      ui.cameraModeBadge.textContent = cameraMode === 'manual' ? 'Manual' : 'Auto';
    } else {
      ui.cameraModeBadge.hidden = true;
    }
  }

  const hasViewResult = Boolean(hasProject && state.reconstructionViewPath);
  ui.viewBtn.disabled = busy || !hasViewResult;
  ui.viewBtn.style.display = hasViewResult ? '' : 'none';
  ui.runBtn.style.display = '';

  if (!busy) {
    const plan = state && state.pipelinePlan;
    if (hasViewResult || (plan && plan.allDone)) {
      ui.runState.textContent = 'Reconstruction complete';
    } else if (plan && plan.anyStarted) {
      ui.runState.textContent = `Resume from ${plan.resumeFrom}`;
    } else if (groupCount > 0) {
      ui.runState.textContent = 'Ready to reconstruct';
    } else {
      ui.runState.textContent = 'Waiting for image folders';
    }
  }
  if (hasProject) {
    ui.projectName.value = state.name || ui.projectName.value;
  }

  renderGroups(hasProject ? state.groups : []);
  updateSteps(groupCount);
}

function renderStageChips(container, plan) {
  if (!container) return;
  container.innerHTML = '';
  if (!plan || !plan.stages) {
    container.hidden = true;
    return;
  }
  container.hidden = false;
  for (const stage of plan.stages) {
    const chip = document.createElement('span');
    chip.className = `stage-chip ${stage.status}`;
    chip.textContent = `${stage.label}: ${stage.status}`;
    container.append(chip);
  }
}

function renderStageStatus(plan) {
  renderStageChips(ui.stageStatus, plan);
}

function renderGroups(groups) {
  ui.groupList.innerHTML = '';
  if (!groups || groups.length === 0) {
    ui.groupList.className = 'group-list empty';
    ui.groupList.textContent = 'No image folders imported yet.';
    return;
  }
  ui.groupList.className = 'group-list';
  const projectManual = state && state.cameraMode === 'manual';
  for (const group of groups) {
    const row = document.createElement('div');
    row.className = 'group-row';

    const top = document.createElement('div');
    top.className = 'group-row-top';

    const title = document.createElement('div');
    title.className = 'group-title';
    title.textContent = group.name;

    const badge = document.createElement('span');
    badge.className = `badge ${projectManual ? 'manual' : 'auto'}`;
    badge.textContent = projectManual ? 'Manual' : 'Auto';

    top.append(title, badge);

    const meta = document.createElement('div');
    meta.className = 'group-meta';
    const size = group.width && group.height ? `${group.width}×${group.height}` : 'size from images';
    meta.textContent = `group_id=${group.groupId}  ${size}  ${group.folder}`;

    row.append(top, meta);
    ui.groupList.append(row);
  }
}

function renderRecent(profile) {
  profileCache = profile;
  ui.recentList.innerHTML = '';
  const items = (profile && profile.recentProjects) || [];
  if (!items.length) {
    ui.recentList.className = 'recent-list empty';
    ui.recentList.textContent = 'No recent projects';
    return;
  }
  ui.recentList.className = 'recent-list';
  for (const item of items) {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'recent-item';
    btn.innerHTML = `<span class="recent-name"></span><span class="recent-path"></span>`;
    btn.querySelector('.recent-name').textContent = item.name || 'Project';
    btn.querySelector('.recent-path').textContent = shortPath(item.workDir);
    btn.title = item.workDir;
    btn.disabled = busy;
    btn.addEventListener('click', () => {
      runAction('Open recent project', () => window.insightAT.openRecent(item.workDir));
    });
    ui.recentList.append(btn);
  }
}

function fillPresetSelect(profile) {
  const presets = (profile && profile.cameraPresets) || [];
  ui.camPresetSelect.innerHTML = '<option value="">— none —</option>';
  for (const preset of presets) {
    const opt = document.createElement('option');
    opt.value = preset.id;
    opt.textContent = preset.name;
    ui.camPresetSelect.append(opt);
  }
}

function updateSteps(groupCount) {
  document.querySelectorAll('.step').forEach((step) => step.classList.remove('active', 'done'));
  const project = document.querySelector('[data-step="project"]');
  const folders = document.querySelector('[data-step="folders"]');
  const reconstruct = document.querySelector('[data-step="reconstruct"]');
  if (!state) {
    project.classList.add('active');
    return;
  }
  project.classList.add('done');
  if (groupCount === 0) {
    folders.classList.add('active');
    return;
  }
  folders.classList.add('done');
  reconstruct.classList.add('active');
}

async function runAction(label, action) {
  if (busy) return;
  let statusAfter = '';
  try {
    setBusy(true, label);
    appendLog(`\n# ${label}\n`);
    const result = await action();
    if (result) applyState(result);
    appendLog(`# Done: ${label}\n`);
    refreshProfile();
  } catch (err) {
    if (err && err.summary) applyState(err.summary);
    else {
      try {
        const s = await window.insightAT.getState();
        if (s) applyState(s);
      } catch (_) {
        /* ignore */
      }
    }
    if (isCancelledError(err)) {
      appendLog(`\n# Stopped: ${label}\n`);
      statusAfter = 'Stopped';
    } else {
      appendLog(`\nERROR: ${err.message || err}\n`);
      statusAfter = 'Failed';
    }
  } finally {
    setBusy(false);
    if (statusAfter) ui.runState.textContent = statusAfter;
  }
}

function isCancelledError(err) {
  if (!err) return false;
  if (err.cancelled) return true;
  return /stopped by user/i.test(String(err.message || err));
}

function showSettingsError(message) {
  if (!message) {
    ui.settingsError.hidden = true;
    ui.settingsError.textContent = '';
    return;
  }
  ui.settingsError.hidden = false;
  ui.settingsError.textContent = message;
}

function showCameraError(message) {
  if (!message) {
    ui.cameraError.hidden = true;
    ui.cameraError.textContent = '';
    ui.cameraError.classList.remove('flash');
    return;
  }
  ui.cameraError.hidden = false;
  ui.cameraError.textContent = message;
}

function flashCameraError() {
  ui.cameraError.classList.remove('flash');
  void ui.cameraError.offsetWidth;
  ui.cameraError.classList.add('flash');
}

function projectCameraMode() {
  return state && state.cameraMode === 'manual' ? 'manual' : 'auto';
}

function fillGroupSelect(selectedId) {
  const groups = (state && state.groups) || [];
  ui.camGroupSelect.innerHTML = '';
  for (const group of groups) {
    const opt = document.createElement('option');
    opt.value = String(group.groupId);
    opt.textContent = group.name;
    ui.camGroupSelect.append(opt);
  }
  const prefer = selectedId != null ? selectedId : cameraTargetGroupId;
  if (prefer != null && groups.some((g) => g.groupId === prefer)) {
    ui.camGroupSelect.value = String(prefer);
    cameraTargetGroupId = prefer;
  } else if (groups[0]) {
    ui.camGroupSelect.value = String(groups[0].groupId);
    cameraTargetGroupId = groups[0].groupId;
  } else {
    cameraTargetGroupId = null;
  }
}

function seedManualForm(group) {
  const src = group || {};
  const defaults = (() => {
    const w = Number(src.width) > 0 ? Number(src.width) : 4000;
    const h = Number(src.height) > 0 ? Number(src.height) : Math.round(w * 0.75);
    const fx = Number(src.fx) > 0 ? Number(src.fx) : Math.round(w * 0.9);
    return { w, h, fx };
  })();

  ui.camFx.value = defaults.fx;
  ui.camFy.value = Number(src.fy) > 0 ? src.fy : defaults.fx;
  ui.camCx.value = src.cx !== undefined && src.cx !== null && src.cx !== ''
    ? src.cx
    : defaults.w / 2;
  ui.camCy.value = src.cy !== undefined && src.cy !== null && src.cy !== ''
    ? src.cy
    : defaults.h / 2;
  ui.camK1.value = src.k1 != null ? src.k1 : 0;
  ui.camK2.value = src.k2 != null ? src.k2 : 0;
  ui.camK3.value = src.k3 != null ? src.k3 : 0;
  ui.camP1.value = src.p1 != null ? src.p1 : 0;
  ui.camP2.value = src.p2 != null ? src.p2 : 0;
  ui.camConvention.value = src.brownConvention === 'opencv' ? 'opencv' : 'context-capture';
  ui.camFix.checked = Boolean(src.fixIntrinsics);
}

function loadSelectedGroupForm() {
  const gid = Number(ui.camGroupSelect.value);
  cameraTargetGroupId = Number.isInteger(gid) ? gid : null;
  const group = ((state && state.groups) || []).find((g) => g.groupId === cameraTargetGroupId);
  seedManualForm(group);
}

function validateCameraForm() {
  const fx = Number(ui.camFx.value);
  if (!(fx > 0)) return 'fx must be a positive number.';
  const fy = ui.camFy.value === '' ? fx : Number(ui.camFy.value);
  if (!(fy > 0)) return 'fy must be a positive number.';
  for (const [key, el] of [
    ['cx', ui.camCx],
    ['cy', ui.camCy],
    ['k1', ui.camK1],
    ['k2', ui.camK2],
    ['k3', ui.camK3],
    ['p1', ui.camP1],
    ['p2', ui.camP2]
  ]) {
    if (el.value === '') continue;
    if (!Number.isFinite(Number(el.value))) return `${key} must be a number.`;
  }
  return '';
}

async function saveSelectedGroupCamera() {
  if (cameraTargetGroupId === null) throw new Error('No group selected.');
  const err = validateCameraForm();
  if (err) {
    const e = new Error(err);
    e.validation = true;
    throw e;
  }
  const result = await window.insightAT.setGroupCamera({
    groupId: cameraTargetGroupId,
    camera: readCameraForm()
  });
  applyState(result);
  appendLog(`# Camera saved for group ${cameraTargetGroupId}\n`);
  return result;
}

function fillSettingsForm(info) {
  ui.settingsBinDir.value = info.binDir || '';
  ui.settingsViewer.value = info.sfmViewerPath || '';
  const lines = [];
  lines.push(`Effective CLI: ${info.resolvedBinDir || '(not found)'}`);
  lines.push(`Effective Viewer: ${info.resolvedViewer || '(not found)'}`);
  lines.push(`Compute: ${info.computeBackend || info.computeMode || 'auto'}`);
  if (info.settingsPath) lines.push(`Config: ${info.settingsPath}`);
  ui.settingsResolved.textContent = lines.join('\n');
  showSettingsError('');
}

async function openSettings() {
  try {
    const info = await window.insightAT.getSettings();
    fillSettingsForm(info);
    applyPathInfo(info);
    ui.settingsModal.hidden = false;
  } catch (err) {
    appendLog(`Settings failed: ${err.message || err}\n`);
  }
}

function closeSettings() {
  ui.settingsModal.hidden = true;
  showSettingsError('');
}

async function openCameraModal() {
  if (!state || !(state.groups || []).length) return;
  ui.cameraTitle.textContent = 'Camera';
  showCameraError('');
  await refreshProfile();
  fillGroupSelect(cameraTargetGroupId);
  loadSelectedGroupForm();
  setCameraModeUi(projectCameraMode());
  ui.cameraModal.hidden = false;
}

function setCameraModeUi(mode) {
  cameraUiMode = mode === 'manual' ? 'manual' : 'auto';
  const isManual = cameraUiMode === 'manual';
  ui.cameraModeAuto.classList.toggle('active', !isManual);
  ui.cameraModeManual.classList.toggle('active', isManual);
  ui.cameraModeAuto.setAttribute('aria-selected', String(!isManual));
  ui.cameraModeManual.setAttribute('aria-selected', String(isManual));
  ui.cameraManualChrome.hidden = !isManual;
  ui.cameraManualPanel.hidden = !isManual;
  ui.cameraSourceLabel.textContent = isManual
    ? 'Project is Manual. Pick a group to edit. Close saves that group.'
    : 'Fully automatic — no manual parameters.';
}

function closeCameraModal() {
  ui.cameraModal.hidden = true;
  showCameraError('');
}

async function requestCloseCameraModal() {
  if (ui.cameraModal.hidden) return;
  if (cameraUiMode === 'auto') {
    closeCameraModal();
    return;
  }

  try {
    showCameraError('');
    setBusy(true, 'Save camera');
    await saveSelectedGroupCamera();
    closeCameraModal();
  } catch (e) {
    showCameraError(e.message || String(e));
    flashCameraError();
  } finally {
    setBusy(false);
  }
}

function readCameraForm() {
  return {
    fx: ui.camFx.value,
    fy: ui.camFy.value,
    cx: ui.camCx.value,
    cy: ui.camCy.value,
    k1: ui.camK1.value,
    k2: ui.camK2.value,
    k3: ui.camK3.value,
    p1: ui.camP1.value,
    p2: ui.camP2.value,
    brownConvention: ui.camConvention.value,
    fixIntrinsics: ui.camFix.checked
  };
}

async function refreshPathsFromCli() {
  try {
    const info = await window.insightAT.getCliInfo();
    applyPathInfo(info);
  } catch (_) {
    /* ignore */
  }
}

async function refreshProfile() {
  try {
    const profile = await window.insightAT.getProfile();
    renderRecent(profile);
    fillPresetSelect(profile);
  } catch (_) {
    /* ignore */
  }
}

ui.createBtn.addEventListener('click', () => {
  runAction('Create project', () => window.insightAT.createProject({
    name: ui.projectName.value.trim() || 'InsightAT_Project'
  }));
});

ui.openBtn.addEventListener('click', () => {
  runAction('Open project', () => window.insightAT.openProject());
});

ui.addFolderBtn.addEventListener('click', () => {
  runAction('Add folder', () => window.insightAT.addFolder({}));
});

ui.runBtn.addEventListener('click', async () => {
  try {
    const plan = await window.insightAT.getPipelinePlan();
    if (!plan.needsChoice) {
      runAction('Run reconstruction', () =>
        window.insightAT.runReconstruction({ mode: plan.defaultMode || 'force' })
      );
      return;
    }
    openRunChoiceModal(plan);
  } catch (err) {
    appendLog(`\nERROR: ${err.message || err}\n`);
    ui.runState.textContent = 'Failed';
  }
});

function openRunChoiceModal(plan) {
  const resumeLabel = (plan.stages.find((s) => s.id === plan.resumeFrom) || {}).label || plan.resumeFrom;
  if (plan.allDone) {
    ui.runChoiceSummary.textContent =
      'All stages are complete. Continue has nowhere to resume — rebuild from Features, or cancel.';
  } else {
    ui.runChoiceSummary.textContent =
      `Progress found. Continue from ${resumeLabel}, or rebuild from Features.`;
  }
  renderStageChips(ui.runChoiceStages, plan);

  ui.runContinueBtn.disabled = Boolean(plan.allDone);
  ui.runContinueBtn.className = plan.defaultMode === 'continue' ? 'primary' : 'secondary';
  ui.runForceBtn.className = plan.defaultMode === 'force' ? 'primary' : 'secondary';

  ui.runChoiceModal.hidden = false;
}

function closeRunChoiceModal() {
  ui.runChoiceModal.hidden = true;
}

ui.runChoiceCloseBtn.addEventListener('click', () => closeRunChoiceModal());
ui.runChoiceModal.addEventListener('click', (event) => {
  if (event.target && event.target.hasAttribute('data-close-run-choice')) closeRunChoiceModal();
});

ui.runContinueBtn.addEventListener('click', () => {
  closeRunChoiceModal();
  runAction('Continue reconstruction', () =>
    window.insightAT.runReconstruction({ mode: 'continue' })
  );
});

ui.runForceBtn.addEventListener('click', () => {
  closeRunChoiceModal();
  runAction('Rebuild reconstruction', () =>
    window.insightAT.runReconstruction({ mode: 'force' })
  );
});

ui.stopBtn.addEventListener('click', async () => {
  if (!busy) return;
  ui.stopBtn.disabled = true;
  ui.runState.textContent = 'Stopping…';
  appendLog('\n# Stop requested\n');
  try {
    await window.insightAT.stopPipeline();
  } catch (err) {
    appendLog(`Stop failed: ${err.message || err}\n`);
  }
});

ui.viewBtn.addEventListener('click', () => {
  window.insightAT.viewReconstruction().catch((err) => {
    appendLog(`View reconstruction failed: ${err.message || err}\n`);
    ui.runState.textContent = err.message || String(err);
  });
});

ui.revealBtn.addEventListener('click', () => {
  window.insightAT.revealWorkDir();
});

ui.clearLogBtn.addEventListener('click', () => {
  clearLogs();
});

if (ui.logTabConsole) {
  ui.logTabConsole.addEventListener('click', () => setLogTab('console'));
}
if (ui.logTabDetail) {
  ui.logTabDetail.addEventListener('click', () => setLogTab('detail'));
}

ui.settingsBtn.addEventListener('click', () => openSettings());
ui.settingsCloseBtn.addEventListener('click', () => closeSettings());
ui.settingsModal.addEventListener('click', (event) => {
  if (event.target && event.target.hasAttribute('data-close-settings')) closeSettings();
});

ui.browseBinBtn.addEventListener('click', async () => {
  const picked = await window.insightAT.pickBinDir();
  if (picked) ui.settingsBinDir.value = picked;
});

ui.browseViewerBtn.addEventListener('click', async () => {
  const picked = await window.insightAT.pickViewer();
  if (picked) ui.settingsViewer.value = picked;
});

ui.settingsClearBtn.addEventListener('click', async () => {
  try {
    showSettingsError('');
    const result = await window.insightAT.setSettings({ binDir: '', sfmViewerPath: '' });
    fillSettingsForm(result.settings);
    applyPathInfo(result.settings);
    if (result.state) applyState(result.state);
    appendLog('# Settings cleared (auto-detect)\n');
  } catch (err) {
    showSettingsError(err.message || String(err));
  }
});

ui.settingsSaveBtn.addEventListener('click', async () => {
  try {
    showSettingsError('');
    const result = await window.insightAT.setSettings({
      binDir: ui.settingsBinDir.value.trim(),
      sfmViewerPath: ui.settingsViewer.value.trim()
    });
    fillSettingsForm(result.settings);
    applyPathInfo(result.settings);
    if (result.state) applyState(result.state);
    appendLog('# Settings saved\n');
    closeSettings();
  } catch (err) {
    showSettingsError(err.message || String(err));
  }
});

ui.cameraBtn.addEventListener('click', () => openCameraModal());
ui.cameraCloseBtn.addEventListener('click', () => requestCloseCameraModal());
ui.cameraModal.addEventListener('click', (event) => {
  if (event.target && event.target.hasAttribute('data-close-camera')) requestCloseCameraModal();
});

ui.camApplyPresetBtn.addEventListener('click', () => {
  const id = ui.camPresetSelect.value;
  const preset = ((profileCache && profileCache.cameraPresets) || []).find((p) => p.id === id);
  if (!preset) return;
  ui.camFx.value = preset.fx;
  ui.camFy.value = preset.fy !== undefined ? preset.fy : '';
  ui.camCx.value = preset.cx !== undefined ? preset.cx : '';
  ui.camCy.value = preset.cy !== undefined ? preset.cy : '';
  ui.camK1.value = preset.k1 || 0;
  ui.camK2.value = preset.k2 || 0;
  ui.camK3.value = preset.k3 || 0;
  ui.camP1.value = preset.p1 || 0;
  ui.camP2.value = preset.p2 || 0;
  ui.camConvention.value = preset.brownConvention === 'opencv' ? 'opencv' : 'context-capture';
});

ui.camSavePresetBtn.addEventListener('click', async () => {
  try {
    showCameraError('');
    const form = readCameraForm();
    const profile = await window.insightAT.saveCameraPreset({
      name: ui.camPresetName.value.trim(),
      ...form
    });
    profileCache = profile;
    fillPresetSelect(profile);
    appendLog(`# Camera preset saved: ${ui.camPresetName.value.trim()}\n`);
  } catch (err) {
    showCameraError(err.message || String(err));
    flashCameraError();
  }
});

ui.cameraModeAuto.addEventListener('click', async () => {
  if (projectCameraMode() === 'auto') {
    setCameraModeUi('auto');
    return;
  }
  try {
    showCameraError('');
    setBusy(true, 'Switch to Auto');
    const result = await window.insightAT.setProjectCameraAuto();
    applyState(result);
    appendLog('# Camera set to Auto for all groups\n');
    setCameraModeUi('auto');
  } catch (err) {
    showCameraError(err.message || String(err));
    flashCameraError();
  } finally {
    setBusy(false);
  }
});

ui.cameraModeManual.addEventListener('click', async () => {
  if (projectCameraMode() === 'manual') {
    fillGroupSelect(cameraTargetGroupId);
    loadSelectedGroupForm();
    setCameraModeUi('manual');
    return;
  }
  try {
    showCameraError('');
    setBusy(true, 'Switch to Manual');
    const result = await window.insightAT.enterCameraManual();
    applyState(result);
    appendLog('# Camera set to Manual; each group seeded with defaults\n');
    fillGroupSelect(cameraTargetGroupId);
    loadSelectedGroupForm();
    setCameraModeUi('manual');
  } catch (err) {
    showCameraError(err.message || String(err));
    flashCameraError();
  } finally {
    setBusy(false);
  }
});

ui.camGroupSelect.addEventListener('change', async () => {
  if (cameraUiMode !== 'manual') return;
  const nextId = Number(ui.camGroupSelect.value);
  if (!Number.isInteger(nextId) || nextId === cameraTargetGroupId) return;

  try {
    showCameraError('');
    setBusy(true, 'Save camera');
    await saveSelectedGroupCamera();
    cameraTargetGroupId = nextId;
    ui.camGroupSelect.value = String(nextId);
    loadSelectedGroupForm();
  } catch (err) {
    // Revert selector if save failed.
    if (cameraTargetGroupId != null) {
      ui.camGroupSelect.value = String(cameraTargetGroupId);
    }
    showCameraError(err.message || String(err));
    flashCameraError();
  } finally {
    setBusy(false);
  }
});

window.insightAT.onLog(appendLog);
window.insightAT.onLogDetail(appendDetailLog);
window.insightAT.onProgress(applyProgress);
window.insightAT.onLogReset((info) => {
  clearLogs();
  resetProgress();
  if (info && info.runId) {
    appendLog(`# Current log run: ${info.runId}\n`);
  }
});
window.insightAT.onPipelinePlan((plan) => {
  if (state) state.pipelinePlan = plan;
  renderStageStatus(plan);
  if (busy && plan) {
    const running = (plan.stages || []).filter((s) => s.status === 'running').map((s) => s.label);
    if (running.length && ui.runState) {
      ui.runState.textContent = `Running: ${running.join(', ')}`;
    }
  }
});
window.insightAT.getState().then((s) => {
  if (s) applyState(s);
  else refreshPathsFromCli();
  refreshProfile();
}).catch(() => {
  refreshPathsFromCli();
  refreshProfile();
});
