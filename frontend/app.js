let appState = null;
let initializedForms = false;
let selectedDeployRootIdx = 0;
let selectedTaskIdx = 0;
let selectedStepIdx = 0;
let lastCamUpdatedAt = 0;

async function apiGet(url) {
  const res = await fetch(url, { cache: "no-store" });
  return await res.json();
}

async function apiPost(url, body) {
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  return await res.json();
}

function setPill(id, text, level) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  el.className = "pill pill-" + (level || "neutral");
}

async function refreshAll(forceRobot = false) {
  try {
    const url = forceRobot ? "/api/state?refresh_robot=1" : "/api/state";
    const data = await apiGet(url);
    if (!data || !data.ok) return;
    appState = data;
    renderTopPills(data);
    renderRobotAndGpus(data);
    renderCamera(data);
    if (!initializedForms) {
      initFormsFromState(data);
      initializedForms = true;
      loadDatasets();
    }
    renderRecordStatus(data);
    renderDeployStatus(data);
    renderLogs(data.logs || []);
  } catch (err) {
    console.error("refresh error:", err);
  }
}

function renderTopPills(data) {
  const r = data.robot || {};
  setPill("pill-nuc", `NUC 4242: ${r.nuc_online ? "ONLINE" : "DOWN"}`, r.nuc_online ? "ok" : "err");
  setPill(
    "pill-desk",
    `Desk 隧道(8443): ${r.desk_tunnel_online ? "已连通" : "未开启"}`,
    r.desk_tunnel_online ? "ok" : "warn"
  );
  setPill(
    "pill-fci",
    `机械臂/FCI: ${r.robot_readable ? "可读 (FCI OK)" : "未就绪"}`,
    r.robot_readable ? "ok" : "warn"
  );
  setPill(
    "pill-sm",
    `SpaceMouse: ${r.spacemouse_connected ? "已连接" : "未检测到"}`,
    r.spacemouse_connected ? "ok" : "warn"
  );
  setPill(
    "pill-cam",
    `Orbbec 相机: ×${r.orbbec_count || 0}`,
    (r.orbbec_count || 0) >= 2 ? "ok" : "warn"
  );

  const deskLink = document.getElementById("desk-link-btn");
  if (deskLink && r.desk_url) {
    deskLink.href = r.desk_url;
  }
}

function renderRobotAndGpus(data) {
  const r = data.robot || {};
  const errEl = document.getElementById("robot-err-banner");
  if (errEl) {
    errEl.textContent = r.error || "";
  }

  const jv = document.getElementById("joint-values");
  if (jv) {
    if (Array.isArray(r.joint_positions) && Array.isArray(r.joint_positions_deg)) {
      jv.textContent = r.joint_positions
        .map((rad, i) => `J${i + 1}: ${r.joint_positions_deg[i].toFixed(1)}° (${rad.toFixed(3)} rad)`)
        .join("\n");
    } else {
      jv.textContent = "等待 NUC / FCI 就绪后显示 7 轴实时关节角…";
    }
  }

  const ev = document.getElementById("ee-values");
  if (ev) {
    if (Array.isArray(r.ee_pose) && r.ee_pose.length === 6) {
      const [x, y, z, rx, ry, rz] = r.ee_pose;
      ev.textContent = `Pos (m) : X=${x.toFixed(4)}  Y=${y.toFixed(4)}  Z=${z.toFixed(4)}\nRot (rad): Rx=${rx.toFixed(3)} Ry=${ry.toFixed(3)} Rz=${rz.toFixed(3)}`;
    } else {
      ev.textContent = "等待末端位姿数据…";
    }
  }

  const gText = document.getElementById("gripper-text");
  const gFill = document.getElementById("gripper-fill");
  if (gText && gFill) {
    if (typeof r.gripper_width_m === "number") {
      const mm = (r.gripper_width_m * 1000).toFixed(1);
      const pct = Math.round((r.gripper_norm || 0) * 100);
      gText.textContent = `${mm} mm (${pct}%)`;
      gFill.style.width = `${pct}%`;
    } else {
      gText.textContent = "--";
      gFill.style.width = "0%";
    }
  }

  const gpuStrip = document.getElementById("gpu-strip");
  if (gpuStrip) {
    const gpus = data.gpus || [];
    gpuStrip.innerHTML = gpus
      .map(
        (g) =>
          `<div class="gpu-chip"><strong>GPU ${g.index}</strong> (${g.name})<br/>显存: ${g.memory_used_mb} / ${g.memory_total_mb} MB · 利用率 ${g.utilization_pct}%</div>`
      )
      .join("");
  }
}

function renderCamera(data) {
  const cam = data.camera || {};
  const statusEl = document.getElementById("cam-status-text");
  if (statusEl) {
    if (cam.ready) {
      const age = typeof cam.age_seconds === "number" ? `${cam.age_seconds.toFixed(1)}s 前` : "";
      statusEl.textContent = `${cam.message || cam.source} (${age})`;
    } else {
      statusEl.textContent = cam.message || "暂无画面";
    }
  }
  if (cam.ready && cam.updated_at && cam.updated_at !== lastCamUpdatedAt) {
    lastCamUpdatedAt = cam.updated_at;
    const ts = Math.floor(cam.updated_at * 1000);
    document.getElementById("img-exterior").src = `/api/camera/exterior.jpg?t=${ts}`;
    document.getElementById("img-wrist").src = `/api/camera/wrist.jpg?t=${ts}`;
  }
}

function initFormsFromState(data) {
  const cfg = data.config || {};
  const selConda = document.getElementById("sel-conda-env");
  const inpConda = document.getElementById("inp-conda-env");
  if (selConda && Array.isArray(cfg.available_conda_envs)) {
    selConda.innerHTML = cfg.available_conda_envs
      .map((e) => `<option value="${e.path}">${e.name}${e.has_franka_cli ? " (含 franka-*)" : ""}</option>`)
      .join("");
    if (cfg.conda_env) {
      selConda.value = cfg.conda_env;
      inpConda.value = cfg.conda_env;
    }
  }

  const recDef = (data.record && data.record.defaults) || {};
  document.getElementById("rec-storage-root").value = cfg.storage_root || "/data2/yjd/.cache/huggingface/lerobot";
  document.getElementById("rec-repo-id").value = recDef.repo_id || "hao/test";
  document.getElementById("rec-prompt").value = recDef.description || "";
  document.getElementById("rec-episodes").value = recDef.num_episodes || 5;
  document.getElementById("rec-fps").value = String(recDef.fps || 30);
  document.getElementById("rec-resume").checked = Boolean(recDef.resume);
  document.getElementById("rec-resume-ds").value = recDef.resume_dataset || "";

  document.getElementById("hil-record-dir").value =
    (cfg.storage_root || "/data2/yjd/.cache/huggingface/lerobot") + "/hil_recordings";

  populateDeploySelectors(data.deploy ? data.deploy.deployments : []);
}

function onCondaSelectChange() {
  const v = document.getElementById("sel-conda-env").value;
  document.getElementById("inp-conda-env").value = v;
}

async function saveCondaEnv() {
  const conda_env = document.getElementById("inp-conda-env").value.trim();
  const storage_root = document.getElementById("rec-storage-root").value.trim();
  await apiPost("/api/config/update", { conda_env, storage_root });
  await refreshAll(true);
}

async function deviceCommand(command) {
  await apiPost("/api/device/command", { command });
  await refreshAll(true);
}

async function confirmAndReset() {
  if (!confirm("确认让机械臂自动运动回 Home 位并打开夹爪？（约需 5 秒，请确认工作台无障碍物且急停在手边）")) {
    return;
  }
  await deviceCommand("reset");
}

async function deskAction(action) {
  const username = document.getElementById("desk-user").value.trim();
  const password = document.getElementById("desk-pass").value;
  await apiPost("/api/device/desk", { action, username, password });
  await refreshAll(true);
}

async function saveDeskAuth() {
  const username = document.getElementById("desk-user").value.trim();
  const password = document.getElementById("desk-pass").value;
  await apiPost("/api/device/desk", {
    action: "status",
    username,
    password,
    save_credentials: true,
  });
  await refreshAll(true);
}

function renderRecordStatus(data) {
  const active = data.record && data.record.active;
  const badge = document.getElementById("record-badge");
  if (!badge) return;
  if (active) {
    badge.textContent = `🔴 正在录制: ${active.extra?.repo_id || active.name} (PID ${active.pid})`;
    badge.className = "pill pill-ok";
  } else {
    badge.textContent = "未在录制";
    badge.className = "pill pill-neutral";
  }
}

async function startRecording() {
  const body = {
    storage_root: document.getElementById("rec-storage-root").value.trim(),
    repo_id: document.getElementById("rec-repo-id").value.trim(),
    description: document.getElementById("rec-prompt").value.trim(),
    num_episodes: parseInt(document.getElementById("rec-episodes").value || "5", 10),
    fps: parseInt(document.getElementById("rec-fps").value || "30", 10),
    resume: document.getElementById("rec-resume").checked,
    resume_dataset: document.getElementById("rec-resume-ds").value.trim(),
  };
  await apiPost("/api/record/start", body);
  await refreshAll(false);
}

async function recordControl(action) {
  await apiPost("/api/record/control", { action });
  await refreshAll(false);
}

async function loadDatasets() {
  const storageRoot = document.getElementById("rec-storage-root").value.trim();
  const wrap = document.getElementById("dataset-list");
  if (!wrap) return;
  const res = await apiGet(`/api/datasets?storage_root=${encodeURIComponent(storageRoot)}`);
  const list = (res && res.datasets) || [];
  if (list.length === 0) {
    wrap.innerHTML = `<div style="padding:10px;" class="muted-text">当前路径 (${storageRoot}) 下暂无已录制数据集。</div>`;
    return;
  }
  wrap.innerHTML = `
    <table class="ds-table">
      <thead>
        <tr>
          <th>数据集 (repo_id)</th>
          <th>类型</th>
          <th>条数</th>
          <th>总帧数</th>
          <th>FPS</th>
          <th>操作</th>
        </tr>
      </thead>
      <tbody>
        ${list
          .map(
            (d) => `
          <tr>
            <td title="${d.prompt || ""}"><strong>${d.repo_id}</strong></td>
            <td>${d.kind === "hil_rollout" ? "HIL/部署" : "LeRobot"}</td>
            <td>${d.total_episodes}</td>
            <td>${d.total_frames}</td>
            <td>${d.fps}</td>
            <td>
              <button class="btn btn-sm btn-primary" onclick="previewDatasetEpisode('${d.path.replace(/'/g, "\\'")}', '${d.repo_id.replace(/'/g, "\\'")}')">👁️ 加载到网页</button>
              <button class="btn btn-sm" onclick="useForResume('${d.repo_id.replace(/'/g, "\\'")}')">续录</button>
              <button class="btn btn-sm btn-warn" onclick="replayDataset('${d.repo_id.replace(/'/g, "\\'")}')">▶️ 回放</button>
            </td>
          </tr>`
          )
          .join("")}
      </tbody>
    </table>
  `;
}

async function previewDatasetEpisode(datasetDir, repoId) {
  const epStr = prompt(`请输入要加载预览的 Episode 编号 (从 0 开始，数据集: ${repoId}):`, "0");
  if (epStr === null) return;
  const episode_index = parseInt(epStr, 10) || 0;
  const detailBox = document.getElementById("dataset-preview-detail");
  if (detailBox) {
    detailBox.style.display = "block";
    detailBox.textContent = `正在从 ${datasetDir} 提取第 ${episode_index} 条样本的相机画面与轨迹数据…`;
  }
  const res = await apiPost("/api/datasets/preview", { dataset_dir: datasetDir, episode_index });
  if (detailBox) {
    if (res && res.ok) {
      const actSample = (res.sample_actions && res.sample_actions[0]) ? JSON.stringify(res.sample_actions[0].map((v) => Number(v.toFixed(4)))) : "无";
      detailBox.innerHTML = `✅ <strong>已加载到上方双相机预览框:</strong> <code>${res.dataset_name}</code> Episode #${res.episode_index} · 共 <strong>${res.frame_count}</strong> 帧 · 首帧动作示例: <code>${actSample}</code>`;
    } else {
      detailBox.textContent = `加载数据集失败: ${(res && res.error) || "未知错误"}`;
    }
  }
  await refreshAll(false);
}

function useForResume(repoId) {
  document.getElementById("rec-resume").checked = true;
  document.getElementById("rec-resume-ds").value = repoId;
  document.getElementById("rec-repo-id").value = repoId.split("_202")[0] || repoId;
}

async function replayDataset(repoId) {
  const epStr = prompt(`在真机上回放 ${repoId} 的哪一条 Episode？(输入编号，从 0 开始):`, "0");
  if (epStr === null) return;
  const episode_index = parseInt(epStr, 10) || 0;
  if (!confirm(`确认在真机上回放 ${repoId} 第 ${episode_index} 条轨迹？机械臂将执行动作！`)) return;
  await apiPost("/api/datasets/replay", { dataset_name: repoId, episode_index, check_only: false });
  await refreshAll(false);
}

function populateDeploySelectors(deployments) {
  const rootSel = document.getElementById("dep-root-select");
  if (!rootSel) return;
  if (!Array.isArray(deployments) || deployments.length === 0) {
    rootSel.innerHTML = `<option value="">未扫描到部署目录</option>`;
    return;
  }
  const prevRootVal = rootSel.value;
  rootSel.innerHTML = deployments
    .map((d, i) => `<option value="${i}">${d.name} (${d.deploy_root})</option>`)
    .join("");
  if (prevRootVal && Number(prevRootVal) < deployments.length) {
    selectedDeployRootIdx = Number(prevRootVal);
    rootSel.value = prevRootVal;
  } else {
    selectedDeployRootIdx = 0;
  }
  updateDeployTaskSelector(deployments);
}

function updateDeployTaskSelector(deployments) {
  const d = deployments[selectedDeployRootIdx];
  const taskSel = document.getElementById("dep-task-select");
  if (!d || !taskSel) return;
  const tasks = d.tasks || [];
  taskSel.innerHTML = tasks
    .map((t, i) => `<option value="${i}">${t.task_id} (GPU ${t.gpu} : Port ${t.port})</option>`)
    .join("");
  if (selectedTaskIdx >= tasks.length) selectedTaskIdx = 0;
  taskSel.value = String(selectedTaskIdx);
  updateDeployStepSelector(deployments);
}

function updateDeployStepSelector(deployments) {
  const d = deployments[selectedDeployRootIdx];
  const t = d && d.tasks && d.tasks[selectedTaskIdx];
  const stepSel = document.getElementById("dep-step-select");
  if (!t || !stepSel) return;
  const ckpts = t.checkpoints || [];
  stepSel.innerHTML = ckpts
    .map((c, i) => `<option value="${i}">step ${c.step}${c.has_weights ? " ✅" : " (未找到权重)"}</option>`)
    .join("");
  if (selectedStepIdx >= ckpts.length) selectedStepIdx = 0;
  stepSel.value = String(selectedStepIdx);
  renderSelectedDeploySummary(deployments);
}

function renderSelectedDeploySummary(deployments) {
  const d = deployments && deployments[selectedDeployRootIdx];
  const t = d && d.tasks && d.tasks[selectedTaskIdx];
  const c = t && t.checkpoints && t.checkpoints[selectedStepIdx];
  if (!d || !t) return;

  document.getElementById("dep-ckpt-path").textContent = c ? c.path : t.checkpoint_base_dir;
  const validEl = document.getElementById("dep-ckpt-valid");
  if (validEl) {
    validEl.textContent = c && c.has_weights ? "权重已就绪" : "权重待检查";
    validEl.className = "pill pill-sm " + (c && c.has_weights ? "pill-ok" : "pill-warn");
  }
  document.getElementById("dep-prompt-text").textContent = t.prompt || "--";
  document.getElementById("dep-gpu-port").textContent = `GPU ${t.gpu} · 127.0.0.1:${t.port} (${t.cam_width}×${t.cam_height} @ ${t.cam_fps}Hz)`;

  const stEl = document.getElementById("dep-server-status");
  if (stEl) {
    if (t.server_status === "ready") {
      stEl.textContent = `🟢 已预加载进 GPU (PID ${t.server_pid || "listening"}) · 点击开始即可秒级部署`;
      stEl.className = "pill pill-ok";
    } else if (t.server_status === "loading") {
      stEl.textContent = `⏳ 正在加载进 GPU (PID ${t.server_pid || "..."})…`;
      stEl.className = "pill pill-warn";
    } else {
      stEl.textContent = "⚪ 未加载进 GPU";
      stEl.className = "pill pill-neutral";
    }
  }
}

function onDeployRootChange() {
  selectedDeployRootIdx = Number(document.getElementById("dep-root-select").value) || 0;
  selectedTaskIdx = 0;
  selectedStepIdx = 0;
  if (appState && appState.deploy) {
    updateDeployTaskSelector(appState.deploy.deployments || []);
  }
}

function onDeployTaskChange() {
  selectedTaskIdx = Number(document.getElementById("dep-task-select").value) || 0;
  selectedStepIdx = 0;
  if (appState && appState.deploy) {
    updateDeployStepSelector(appState.deploy.deployments || []);
  }
}

function onDeployStepChange() {
  selectedStepIdx = Number(document.getElementById("dep-step-select").value) || 0;
  if (appState && appState.deploy) {
    renderSelectedDeploySummary(appState.deploy.deployments || []);
  }
}

async function promptAddDeployRoot() {
  const p = prompt("请输入部署脚本所在目录绝对路径:", "/data2/yjd/workspace/Franka-Teleop/");
  if (!p) return;
  await apiPost("/api/config/update", { custom_deploy_root: p.trim() });
  initializedForms = false;
  await refreshAll(false);
}

function getCurrentDeploySelection() {
  const deployments = (appState && appState.deploy && appState.deploy.deployments) || [];
  const d = deployments[selectedDeployRootIdx];
  const t = d && d.tasks && d.tasks[selectedTaskIdx];
  const c = t && t.checkpoints && t.checkpoints[selectedStepIdx];
  if (!d || !t) return null;
  return {
    deploy_root: d.deploy_root,
    task_id: t.task_id,
    checkpoint_step: c ? c.step : t.default_step,
    checkpoint_path: c ? c.path : "",
    prompt: t.prompt,
    cam_width: t.cam_width,
    cam_height: t.cam_height,
    fps: t.cam_fps,
  };
}

async function preloadDeployToGpu() {
  const sel = getCurrentDeploySelection();
  if (!sel) return;
  await apiPost("/api/deploy/preload", {
    deploy_root: sel.deploy_root,
    task_id: sel.task_id,
    checkpoint_step: sel.checkpoint_step,
  });
  await refreshAll(false);
}

async function unloadDeployFromGpu() {
  const sel = getCurrentDeploySelection();
  if (!sel) return;
  await apiPost("/api/deploy/unload", {
    deploy_root: sel.deploy_root,
    task_id: sel.task_id,
  });
  await refreshAll(false);
}

async function startDeployment() {
  const sel = getCurrentDeploySelection();
  if (!sel) return;
  const body = {
    ...sel,
    launch_mode: document.getElementById("dep-launch-mode").value,
    enable_hil: document.getElementById("hil-enable").checked,
    auto_resume: document.getElementById("hil-auto-resume").checked,
    idle_resume_sec: parseFloat(document.getElementById("hil-idle-sec").value || "1.2"),
    record_mode: document.getElementById("hil-record-mode").value,
    record_dir: document.getElementById("hil-record-dir").value.trim(),
  };
  const res = await apiPost("/api/deploy/start", body);
  if (res && res.need_preload) {
    if (confirm(res.message + "\n\n是否现在立即帮您启动『预加载模型进 GPU』？")) {
      await preloadDeployToGpu();
    }
  }
  await refreshAll(false);
}

async function controlDeployment(action) {
  await apiPost("/api/deploy/control", { action });
  await refreshAll(false);
}

async function syncHilOptionsLive() {
  if (!appState || !appState.deploy || !appState.deploy.active_rollout) return;
  await apiPost("/api/deploy/control", {
    action: "update_options",
    enable_hil: document.getElementById("hil-enable").checked,
    auto_resume: document.getElementById("hil-auto-resume").checked,
    idle_resume_sec: parseFloat(document.getElementById("hil-idle-sec").value || "1.2"),
    record_mode: document.getElementById("hil-record-mode").value,
  });
}

function renderDeployStatus(data) {
  const dep = data.deploy || {};
  if (Array.isArray(dep.deployments)) {
    renderSelectedDeploySummary(dep.deployments);
  }

  const active = dep.active_rollout;
  const live = dep.rollout_status || {};
  const badge = document.getElementById("deploy-mode-badge");
  const banner = document.getElementById("rollout-live-banner");

  if (active) {
    const mode = (live.mode || "policy").toLowerCase();
    if (mode === "hil") {
      badge.textContent = `🕹️ HIL 人工接管录制中 (HIL ${live.hil_steps || 0} 步)`;
      badge.className = "pill pill-warn";
    } else if (mode === "paused") {
      badge.textContent = "⏸️ 已暂停推理 (点击继续或移动 SpaceMouse)";
      badge.className = "pill pill-err";
    } else {
      badge.textContent = `🤖 模型推理中 (PID ${active.pid} · 步数 ${live.step || 0})`;
      badge.className = "pill pill-ok";
    }
    if (banner) {
      banner.textContent =
        live.message ||
        `运行中 (PID ${active.pid}) · Policy ${live.policy_steps || 0} 步 · HIL ${live.hil_steps || 0} 步` +
          (live.active_record_path ? ` · 录制目录: ${live.active_record_path}` : "");
    }
  } else {
    badge.textContent = "待机";
    badge.className = "pill pill-neutral";
    if (banner) {
      banner.textContent =
        "当前无活动推理进程。可先点击『1. 预加载模型进 GPU』，就绪后点击『2. 点击开始部署推理』（运行中支持 Alt+P 键或按钮暂停/继续，以及移动 SpaceMouse 自动打断录制 HIL）。";
    }
  }
}

function renderLogs(lines) {
  const el = document.getElementById("log-console");
  if (!el) return;
  const atBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 40;
  el.textContent = lines.length ? lines.join("\n") : "暂无日志输出…";
  if (atBottom) {
    el.scrollTop = el.scrollHeight;
  }
}

function clearLogView() {
  const el = document.getElementById("log-console");
  if (el) el.textContent = "";
}

// Keyboard shortcut: Alt+P toggles deployment pause/resume
window.addEventListener("keydown", (e) => {
  if (e.altKey && e.key.toLowerCase() === "p") {
    e.preventDefault();
    controlDeployment("toggle_pause");
  }
});

refreshAll(false);
setInterval(() => refreshAll(false), 1500);
