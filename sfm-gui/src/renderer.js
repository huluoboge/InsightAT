'use strict';

const $ = (id) => document.getElementById(id);

const ui = {
  projectName: $('projectName'),
  createBtn: $('createBtn'),
  openBtn: $('openBtn'),
  addFolderBtn: $('addFolderBtn'),
  runBtn: $('runBtn'),
  viewBtn: $('viewBtn'),
  revealBtn: $('revealBtn'),
  clearLogBtn: $('clearLogBtn'),
  logOutput: $('logOutput'),
  projectTitle: $('projectTitle'),
  projectSubtitle: $('projectSubtitle'),
  workDir: $('workDir'),
  groupCount: $('groupCount'),
  imageCount: $('imageCount'),
  cliBinDir: $('cliBinDir'),
  folderCount: $('folderCount'),
  groupList: $('groupList'),
  runState: $('runState')
};

let busy = false;
let state = null;

function appendLog(text) {
  ui.logOutput.textContent += text;
  ui.logOutput.scrollTop = ui.logOutput.scrollHeight;
}

function setBusy(nextBusy, label) {
  busy = nextBusy;
  document.body.classList.toggle('busy', busy);
  for (const button of [ui.createBtn, ui.openBtn, ui.addFolderBtn, ui.runBtn, ui.viewBtn, ui.revealBtn]) {
    button.disabled = busy;
  }
  applyState(state);
  if (label) ui.runState.textContent = label;
}

function shortPath(value) {
  if (!value) return 'Not selected';
  if (value.length <= 46) return value;
  return `...${value.slice(-43)}`;
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
    ui.cliBinDir.textContent = shortPath(cliPath);
    ui.cliBinDir.title = cliPath;
  } else if (state && state.cliFound === false) {
    ui.cliBinDir.textContent = 'Not found (build CLI first)';
    ui.cliBinDir.title = '';
  }

  ui.addFolderBtn.disabled = busy || !hasProject;
  ui.runBtn.disabled = busy || !hasProject || groupCount === 0;
  ui.revealBtn.disabled = busy || !hasProject;

  const hasViewResult = Boolean(hasProject && state.reconstructionViewPath);
  ui.viewBtn.disabled = busy || !hasViewResult;
  ui.viewBtn.style.display = hasViewResult ? '' : 'none';
  ui.runBtn.style.display = '';

  ui.runState.textContent = hasViewResult
    ? 'Reconstruction complete'
    : (groupCount > 0 ? 'Ready to reconstruct' : 'Waiting for image folders');
  if (hasProject) {
    ui.projectName.value = state.name || ui.projectName.value;
  }

  renderGroups(hasProject ? state.groups : []);
  updateSteps(groupCount);
}

function renderGroups(groups) {
  ui.groupList.innerHTML = '';
  if (!groups || groups.length === 0) {
    ui.groupList.className = 'group-list empty';
    ui.groupList.textContent = 'No image folders imported yet.';
    return;
  }
  ui.groupList.className = 'group-list';
  for (const group of groups) {
    const row = document.createElement('div');
    row.className = 'group-row';
    const title = document.createElement('div');
    title.className = 'group-title';
    title.textContent = group.name;
    const meta = document.createElement('div');
    meta.className = 'group-meta';
    meta.textContent = `group_id=${group.groupId}  ${group.folder}`;
    row.append(title, meta);
    ui.groupList.append(row);
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
  try {
    setBusy(true, label);
    appendLog(`\n# ${label}\n`);
    const result = await action();
    if (result) applyState(result);
    appendLog(`# Done: ${label}\n`);
  } catch (err) {
    appendLog(`\nERROR: ${err.message || err}\n`);
    ui.runState.textContent = 'Failed';
  } finally {
    setBusy(false);
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

ui.runBtn.addEventListener('click', () => {
  runAction('Run reconstruction', () => window.insightAT.runReconstruction({}));
});

ui.viewBtn.addEventListener('click', () => {
  window.insightAT.viewReconstruction();
});

ui.revealBtn.addEventListener('click', () => {
  window.insightAT.revealWorkDir();
});

ui.clearLogBtn.addEventListener('click', () => {
  ui.logOutput.textContent = '';
});

window.insightAT.onLog(appendLog);
window.insightAT.getState().then((s) => {
  if (s) applyState(s);
  else {
    window.insightAT.getCliInfo().then((info) => {
      ui.cliBinDir.textContent = info.path ? shortPath(info.path) : 'Not found (build CLI first)';
      ui.cliBinDir.title = info.path || '';
    }).catch(() => {
      ui.cliBinDir.textContent = 'Not found';
    });
  }
});
