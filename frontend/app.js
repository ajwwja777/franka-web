let globalState = null;
let currentView = "console";
let lastCamSeq = -1;
let lastCamUpdatedAt = 0;
let camImgLoading = false;
let recordDefaultsLoaded = false;
let currentPreviewDataset = null;
let clearedLogCount = 0;

function switchView(viewName) {
  currentView = viewName;
  document.querySelectorAll(".module-nav .nav-item").forEach((btn) => {
    btn.classList.toggle("active", btn.getAttribute("data-view") === viewName);
  });
  document.querySelectorAll(".workbench-view").forEach((v) => {
    v.classList.toggle("active", v.id === `view-${viewName}`);
  });
  if (viewName === "record") {
    loadDatasetsList();
  }
}

function setLastOp(badge, text) {
  const b = document.getElementById("sideLastOpBadge");
  const t = document.getElementById("sideLastOpText");
  if (b) b.textContent = badge;
  if (t) t.textContent = text;
}

function setDot(id, status) {
  const el = document.getElementById(id);
  if (!el) return;
  el.className = "dot " + (status || "");
}

// ==================== 1. 实时相机高频轮询 (180ms, ~5.5 FPS Web 刷新 + 30Hz 采集显示) ====================
async function pollCameraRealtime() {
  try {
    const res = await fetch("/api/camera/meta");
    if (!res.ok) return;
    const data = await res.json();
    if (data && data.camera) {
      renderCameraInfo(data.camera);
    }
  } catch (_) {}
}

function renderCameraInfo(cam) {
  const hz = typeof cam.capture_hz === "number" ? cam.capture_hz : 0;
  const hzStr = hz > 0 ? `${hz.toFixed(1)} Hz` : "0.0 Hz";
  const fpsStr = hz > 0 ? `${hz.toFixed(1)} FPS` : "-- FPS";
  const w = cam.width || 424;
  const h = cam.height || 240;
  const seq = cam.frame_seq || 0;
  const extMs = typeof cam.exterior_ms === "number" ? `${cam.exterior_ms.toFixed(1)} ms` : "-- ms";
  const wristMs = typeof cam.wrist_ms === "number" ? `${cam.wrist_ms.toFixed(1)} ms` : "-- ms";

  const hzPill = document.getElementById("camLiveHzPill");
  if (hzPill) {
    hzPill.textContent = `实时采集: ${hzStr} (目标 ${cam.target_fps || 30} Hz)`;
    hzPill.className = hz > 1.0 ? "hz-live-pill" : "hz-live-pill idle";
  }

  const srcBadge = document.getElementById("camSourceBadge");
  if (srcBadge) {
    const modeMap = {
      STREAM: "实时流预览",
      RECORD: "录制实况",
      POLICY: "策略推理实况",
      HIL: "SpaceMouse HIL 实况",
      PAUSED: "部署暂停实况",
    };
    srcBadge.textContent = modeMap[cam.mode] || cam.source || "待机";
  }

  const btnStream = document.getElementById("btnToggleCamStream");
  if (btnStream) {
    if (cam.stream_running) {
      btnStream.textContent = "⏹ 停止实时相机";
      btnStream.className = "btn btn-xs btn-warn";
    } else {
      btnStream.textContent = "▶ 开启实时相机";
      btnStream.className = "btn btn-xs btn-primary";
    }
  }

  const extIdEl = document.getElementById("camExtId");
  if (extIdEl && cam.exterior_id) extIdEl.textContent = cam.exterior_id;
  const wristIdEl = document.getElementById("camWristId");
  if (wristIdEl && cam.wrist_id) wristIdEl.textContent = cam.wrist_id;

  const extStats = document.getElementById("camExtStats");
  if (extStats) {
    extStats.textContent = `${fpsStr} · ${w}×${h} · #${seq} · ${extMs}`;
  }
  const wristStats = document.getElementById("camWristStats");
  if (wristStats) {
    wristStats.textContent = `${fpsStr} · ${w}×${h} · #${seq} · ${wristMs}`;
  }

  const isFresh = cam.ready && cam.age_seconds >= 0 && cam.age_seconds < 4.5;
  setDot("dotCamExt", isFresh ? "ok" : cam.ready ? "warn" : "");
  setDot("dotCamWrist", isFresh ? "ok" : cam.ready ? "warn" : "");
  setDot("subDotCam", isFresh ? "ok" : cam.ready ? "warn" : "err");

  const armCamHz = document.getElementById("armCamHzVal");
  if (armCamHz) armCamHz.textContent = hzStr;
  const topoCamMeta = document.getElementById("topoCamMeta");
  if (topoCamMeta) topoCamMeta.textContent = isFresh ? `实时 ${hzStr}` : "待机 / 点击开启";
  const topoNodeCam = document.getElementById("topoNodeCam");
  if (topoNodeCam) topoNodeCam.className = "topo-node " + (isFresh ? "ok" : "");

  // 当 frame_seq 或 updated_at 变化时无闪烁刷新双相机图像
  if (cam.ready && (seq !== lastCamSeq || cam.updated_at !== lastCamUpdatedAt) && !camImgLoading) {
    lastCamSeq = seq;
    lastCamUpdatedAt = cam.updated_at;
    camImgLoading = true;
    const ts = `${seq}_${Math.floor((cam.updated_at || Date.now()) * 1000)}`;
    const imgExt = document.getElementById("imgExterior");
    const imgWrist = document.getElementById("imgWrist");
    let loaded = 0;
    const done = () => {
      loaded++;
      if (loaded >= 2) camImgLoading = false;
    };
    if (imgExt) {
      imgExt.onload = done;
      imgExt.onerror = done;
      imgExt.src = `/api/camera/exterior?t=${ts}`;
    } else {
      done();
    }
    if (imgWrist) {
      imgWrist.onload = done;
      imgWrist.onerror = done;
      imgWrist.src = `/api/camera/wrist?t=${ts}`;
    } else {
      done();
    }
    setTimeout(() => {
      camImgLoading = false;
    }, 450);
  }
}

async function toggleCameraStream() {
  setLastOp("CAMERA", "切换双相机实时流...");
  try {
    const res = await fetch("/api/camera/stream", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action: "toggle" }),
    });
    const data = await res.json();
    setLastOp(data.ok ? "OK" : "WARN", data.message || "相机流状态已更新");
    setTimeout(pollCameraRealtime, 300);
  } catch (e) {
    setLastOp("ERR", `切换相机流失败: ${e}`);
  }
}

// ==================== 2. 全局状态轮询 (2000ms) ====================
async function refreshState(forceRobot = false) {
  try {
    const url = forceRobot ? "/api/state?refresh_robot=1" : "/api/state";
    const res = await fetch(url);
    if (!res.ok) return;
    const data = await res.json();
    globalState = data;
    renderAllState(data);
  } catch (e) {
    const sideBadge = document.getElementById("sideDevBadge");
    if (sideBadge) sideBadge.textContent = "离线";
  }
}

function renderAllState(data) {
  const r = data.robot || {};
  const cam = data.camera || {};
  const rec = data.record || {};
  const dep = data.deploy || {};

  // 1) 左侧边栏下部状态卡片
  const sideDevBadge = document.getElementById("sideDevBadge");
  const sideDevText = document.getElementById("sideDevText");
  const sideDevTime = document.getElementById("sideDevTime");
  if (sideDevBadge) {
    const allOk = r.nuc_online && r.robot_readable;
    sideDevBadge.textContent = allOk ? "正常" : r.nuc_online ? "部分就绪" : "待连接";
    sideDevBadge.className = "mini-pill " + (allOk ? "ok" : "warn");
  }
  if (sideDevText) {
    sideDevText.textContent = `NUC:${r.nuc_online ? "在线" : "离线"} · 机械臂:${r.robot_readable ? "可读" : "未连接"} · 相机:${(cam.capture_hz || 0).toFixed(1)}Hz`;
  }
  if (sideDevTime) {
    sideDevTime.textContent = `更新于 ${new Date().toLocaleTimeString()} · 探测 ${r.probe_ms || "--"} ms`;
  }

  // 2) Conda 下拉框
  const condaSel = document.getElementById("condaEnvSelect");
  if (condaSel && data.config && Array.isArray(data.config.available_conda_envs)) {
    const currentVal = data.config.conda_env;
    if (condaSel.options.length !== data.config.available_conda_envs.length) {
      condaSel.innerHTML = "";
      data.config.available_conda_envs.forEach((env) => {
        const opt = document.createElement("option");
        opt.value = env.path;
        opt.textContent = `${env.name}${env.has_franka_cli ? " ★" : ""}`;
        condaSel.appendChild(opt);
      });
    }
    condaSel.value = currentVal;
  }

  // 3) 拓扑图节点与右侧子系统指示灯
  const topoDesk = document.getElementById("topoNodeDesk");
  if (topoDesk) topoDesk.className = "topo-node " + (r.desk_tunnel_online ? "ok" : "warn");
  const topoDeskMeta = document.getElementById("topoDeskMeta");
  if (topoDeskMeta) topoDeskMeta.textContent = r.desk_tunnel_online ? "8443 隧道已连通" : "8443 未开启 (点 connect)";

  const topoNuc = document.getElementById("topoNodeNuc");
  if (topoNuc) topoNuc.className = "topo-node " + (r.nuc_online ? "ok" : "warn");
  const topoNucMeta = document.getElementById("topoNucMeta");
  if (topoNucMeta) topoNucMeta.textContent = r.nuc_online ? `${r.nuc_ip}:4242 在线` : `${r.nuc_ip}:4242 离线`;

  const topoArm = document.getElementById("topoNodeArm");
  if (topoArm) topoArm.className = "topo-node " + (r.robot_readable ? "ok" : "warn");
  const topoArmMeta = document.getElementById("topoArmMeta");
  if (topoArmMeta) {
    topoArmMeta.textContent = r.robot_readable
      ? `7DoF 正常 · 夹爪 ${r.gripper_width !== null && r.gripper_width !== undefined ? (r.gripper_width * 1000).toFixed(1) + "mm" : "--"}`
      : r.error || "等待连接";
  }

  setDot("subDotArm", r.robot_readable ? "ok" : "err");
  setDot("subDotNuc", r.nuc_online ? "ok" : "err");
  setDot("subDotDesk", r.desk_tunnel_online ? "ok" : "warn");
  setDot("subDotSm", "ok");
  setDot("subDotRec", rec.active ? "ok" : "");
  setDot("subDotDeploy", dep.active_rollout ? "ok" : "");

  // 4) 机械臂关节与末端位姿面板
  const armReadableVal = document.getElementById("armReadableVal");
  if (armReadableVal) {
    armReadableVal.textContent = r.robot_readable ? "● 在线可读" : "○ 未就绪";
    armReadableVal.style.color = r.robot_readable ? "#059669" : "#d97706";
  }
  const armGripperVal = document.getElementById("armGripperVal");
  if (armGripperVal) {
    if (typeof r.gripper_width === "number") {
      armGripperVal.textContent = `${(r.gripper_width * 1000).toFixed(1)} mm (${((r.gripper_norm || 0) * 100).toFixed(0)}%)`;
    } else {
      armGripperVal.textContent = "-- mm";
    }
  }
  const armReadLatency = document.getElementById("armReadLatency");
  if (armReadLatency) {
    armReadLatency.textContent = `只读 zerorpc · ${r.probe_ms || "--"} ms`;
  }

  if (Array.isArray(r.joint_positions) && r.joint_positions.length === 7) {
    const cells = document.querySelectorAll("#jointGrid .joint-cell");
    r.joint_positions.forEach((rad, i) => {
      const deg = Array.isArray(r.joint_positions_deg) ? r.joint_positions_deg[i] : (rad * 180) / Math.PI;
      if (cells[i]) {
        cells[i].innerHTML = `<span class="j-name">J${i}</span><span class="j-val">${rad.toFixed(3)} rad (${deg.toFixed(1)}°)</span>`;
      }
    });
  }

  if (Array.isArray(r.ee_pose) && r.ee_pose.length === 6) {
    const labels = ["X (m)", "Y (m)", "Z (m)", "RX (rad)", "RY (rad)", "RZ (rad)"];
    const cells = document.querySelectorAll("#eeGrid .joint-cell");
    r.ee_pose.forEach((v, i) => {
      if (cells[i]) {
        cells[i].innerHTML = `<span class="j-name">${labels[i]}</span><span class="j-val">${v.toFixed(4)}</span>`;
      }
    });
  }

  // 5) 录制默认值与状态
  if (!recordDefaultsLoaded && rec.defaults) {
    recordDefaultsLoaded = true;
    if (rec.defaults.repo_id) document.getElementById("recRepoId").value = rec.defaults.repo_id;
    if (rec.defaults.description) document.getElementById("recDescription").value = rec.defaults.description;
    if (rec.defaults.num_episodes) document.getElementById("recEpisodes").value = rec.defaults.num_episodes;
    if (rec.defaults.fps) document.getElementById("recFps").value = rec.defaults.fps;
    if (data.config && data.config.storage_root) {
      document.getElementById("recStorageRoot").value = data.config.storage_root;
    }
  }
  const recPill = document.getElementById("recStatusPill");
  if (recPill) {
    if (rec.active) {
      recPill.textContent = `● 录制中 (PID ${rec.active.pid})`;
      recPill.className = "mini-pill ok";
    } else {
      recPill.textContent = "未在录制";
      recPill.className = "mini-pill";
    }
  }

  // 6) 部署目录、任务、Checkpoint 与 GPU 预热状态
  renderDeploymentSelectors(dep.deployments || []);
  renderRolloutLiveStatus(dep.active_rollout, dep.rollout_status || {}, cam);

  // 7) GPU 卡片
  renderGpuCards(data.gpus || []);

  // 8) 右侧终端输出日志
  renderLogs(data.logs || []);
}

// ==================== 3. 设备控制与 Desk 动作 ====================
async function runDeviceCommand(command) {
  setLastOp("EXEC", `正在执行 ${command}...`);
  try {
    const res = await fetch("/api/device/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command }),
    });
    const data = await res.json();
    setLastOp(data.ok ? "OK" : "WARN", data.message || `${command} 完成`);
    refreshState(true);
  } catch (e) {
    setLastOp("ERR", `执行 ${command} 失败: ${e}`);
  }
}

function confirmResetRobot() {
  if (!confirm("确认执行 `franka-reset` 让机械臂回到 Home 位并打开夹爪吗？请确保机械臂周围无障碍物。")) {
    return;
  }
  runDeviceCommand("reset");
}

async function runDeskAction(action) {
  const username = document.getElementById("deskUser")?.value || "";
  const password = document.getElementById("deskPass")?.value || "";
  setLastOp("DESK", `正在执行 Desk ${action}...`);
  try {
    const res = await fetch("/api/device/desk", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action,
        username: username || null,
        password: password || null,
        save_credentials: Boolean(username && password),
      }),
    });
    const data = await res.json();
    if (data.need_credentials) {
      toggleSettingsModal(true);
      alert(data.message || "请先在『⚙ 设置』中填写 Franka Desk 用户名与密码，或点击『打开 Desk 网页』手动操作。");
    }
    setLastOp(data.ok ? "OK" : "WARN", data.message || `Desk ${action} 已返回`);
    refreshState(true);
  } catch (e) {
    setLastOp("ERR", `Desk 操作异常: ${e}`);
  }
}

async function onCondaSelectChange() {
  const val = document.getElementById("condaEnvSelect").value;
  await fetch("/api/config/update", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ conda_env: val }),
  });
  setLastOp("CONDA", `已切换环境: ${val}`);
  refreshState(true);
}

// ==================== 4. 数据采集与已有数据集预览 ====================
document.addEventListener("change", (e) => {
  if (e.target && e.target.id === "recResume") {
    const wrap = document.getElementById("recResumeDatasetWrap");
    if (wrap) wrap.style.display = e.target.value === "true" ? "flex" : "none";
  }
});

async function startRecording() {
  const payload = {
    repo_id: document.getElementById("recRepoId").value.trim(),
    storage_root: document.getElementById("recStorageRoot").value.trim(),
    description: document.getElementById("recDescription").value.trim(),
    num_episodes: parseInt(document.getElementById("recEpisodes").value || "5", 10),
    fps: parseInt(document.getElementById("recFps").value || "30", 10),
    resume: document.getElementById("recResume").value === "true",
    resume_dataset: document.getElementById("recResumeDataset").value.trim(),
  };
  setLastOp("RECORD", `启动录制 ${payload.repo_id}...`);
  const res = await fetch("/api/record/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await res.json();
  setLastOp(data.ok ? "RECORD" : "WARN", data.message || "录制请求已返回");
  refreshState();
}

async function sendRecordControl(action) {
  const res = await fetch("/api/record/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action }),
  });
  const data = await res.json();
  setLastOp("RECORD", data.message || action);
  refreshState();
}

async function loadDatasetsList() {
  const storageRoot = document.getElementById("recStorageRoot")?.value?.trim() || "";
  const tbody = document.getElementById("datasetTableBody");
  if (!tbody) return;
  try {
    const res = await fetch(`/api/datasets?storage_root=${encodeURIComponent(storageRoot)}`);
    const data = await res.json();
    const list = data.datasets || [];
    if (list.length === 0) {
      tbody.innerHTML = `<tr><td colspan="5" class="muted">当前存储路径下暂无数据集</td></tr>`;
      return;
    }
    tbody.innerHTML = list
      .map(
        (ds) => `
      <tr>
        <td class="mono"><strong>${ds.repo_id}</strong></td>
        <td><span class="mini-pill">${ds.kind}</span></td>
        <td class="mono">${ds.total_episodes} ep / ${ds.total_frames} f</td>
        <td class="mono">${ds.fps}</td>
        <td>
          <button class="btn btn-xs btn-primary" onclick='previewDatasetInWeb(${JSON.stringify(ds.path)}, ${JSON.stringify(ds.repo_id)}, 0)'>加载到网页</button>
          <button class="btn btn-xs" onclick='setAsResumeTarget(${JSON.stringify(ds.repo_id)})'>续采</button>
        </td>
      </tr>
    `
      )
      .join("");
  } catch (e) {
    tbody.innerHTML = `<tr><td colspan="5" class="muted">加载数据集列表失败: ${e}</td></tr>`;
  }
}

function setAsResumeTarget(repoId) {
  const shortName = repoId.includes("/") ? repoId.split("/").pop() : repoId;
  document.getElementById("recResume").value = "true";
  document.getElementById("recResumeDatasetWrap").style.display = "flex";
  document.getElementById("recResumeDataset").value = shortName;
  setLastOp("RESUME", `已设置续采目标: ${shortName}`);
}

async function previewDatasetInWeb(datasetDir, repoId, episodeIdx) {
  currentPreviewDataset = { datasetDir, repoId };
  const box = document.getElementById("datasetPreviewBox");
  if (box) box.style.display = "flex";
  document.getElementById("dpTitle").textContent = `数据集预览: ${repoId}`;
  document.getElementById("dpEpisodeIdx").value = episodeIdx;
  document.getElementById("dpMeta").textContent = "正在解析轨迹与抽帧图像...";
  document.getElementById("dpImages").innerHTML = "";

  const res = await fetch("/api/datasets/preview", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ dataset_dir: datasetDir, episode_index: episodeIdx }),
  });
  const data = await res.json();
  if (!data.ok) {
    document.getElementById("dpMeta").textContent = `加载失败: ${data.error || "未知错误"}`;
    return;
  }
  document.getElementById("dpMeta").textContent = `路径: ${data.dataset_dir} | 总条数: ${data.total_episodes} | 当前条帧数: ${data.episode_frames} | Prompt: ${data.prompt || "--"}`;
  const imgs = data.preview_images || [];
  document.getElementById("dpImages").innerHTML = imgs
    .map(
      (im) => `
    <div>
      <img src="data:image/jpeg;base64,${im.b64}" alt="${im.cam}" />
      <div class="muted mono">${im.cam} (${im.pos})</div>
    </div>
  `
    )
    .join("");
}

function reloadCurrentDatasetPreview() {
  if (!currentPreviewDataset) return;
  const ep = parseInt(document.getElementById("dpEpisodeIdx").value || "0", 10);
  previewDatasetInWeb(currentPreviewDataset.datasetDir, currentPreviewDataset.repoId, ep);
}

async function replayCurrentDataset(checkOnly) {
  if (!currentPreviewDataset) return;
  const ep = parseInt(document.getElementById("dpEpisodeIdx").value || "0", 10);
  if (!checkOnly && !confirm(`确认在真机上回放 ${currentPreviewDataset.repoId} 的第 ${ep} 条轨迹吗？`)) {
    return;
  }
  const res = await fetch("/api/datasets/replay", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      dataset_name: currentPreviewDataset.repoId,
      episode_index: ep,
      check_only: checkOnly,
    }),
  });
  const data = await res.json();
  setLastOp("REPLAY", data.message || "已触发回放");
}

// ==================== 5. 部署、Checkpoint 路径显示、GPU 预加载与 SpaceMouse HIL ====================
let lastDeployRootsSig = "";

function renderDeploymentSelectors(deployments) {
  const rootSel = document.getElementById("deployRootSelect");
  if (!rootSel || !Array.isArray(deployments) || deployments.length === 0) return;

  const sig = deployments.map((d) => `${d.deploy_root}:${(d.tasks || []).map((t) => `${t.task_id}:${t.server_status}`).join(",")}`).join("|");
  const prevRoot = rootSel.value;
  const prevTask = document.getElementById("deployTaskSelect")?.value;
  const prevStep = document.getElementById("deployStepSelect")?.value;

  if (sig !== lastDeployRootsSig) {
    lastDeployRootsSig = sig;
    rootSel.innerHTML = deployments
      .map((d) => `<option value="${d.deploy_root}">${d.name || d.deploy_root} (${d.deploy_root})</option>`)
      .join("");
    if (prevRoot && deployments.some((d) => d.deploy_root === prevRoot)) {
      rootSel.value = prevRoot;
    }
    onDeployRootChange(prevTask, prevStep);
  } else {
    updateSelectedTaskMeta();
  }
}

function getSelectedDeployObj() {
  const rootVal = document.getElementById("deployRootSelect")?.value;
  const list = globalState?.deploy?.deployments || [];
  return list.find((d) => d.deploy_root === rootVal) || list[0] || null;
}

function getSelectedTaskObj() {
  const depObj = getSelectedDeployObj();
  if (!depObj) return null;
  const taskVal = document.getElementById("deployTaskSelect")?.value;
  return (depObj.tasks || []).find((t) => t.task_id === taskVal) || (depObj.tasks || [])[0] || null;
}

function onDeployRootChange(preferredTask, preferredStep) {
  const depObj = getSelectedDeployObj();
  const taskSel = document.getElementById("deployTaskSelect");
  if (!depObj || !taskSel) return;
  const tasks = depObj.tasks || [];
  taskSel.innerHTML = tasks
    .map((t) => `<option value="${t.task_id}">${t.task_id} [${t.server_status === "ready" ? "★ GPU已预热" : t.server_status}]</option>`)
    .join("");
  if (preferredTask && tasks.some((t) => t.task_id === preferredTask)) {
    taskSel.value = preferredTask;
  }
  onDeployTaskChange(preferredStep);
}

function onDeployTaskChange(preferredStep) {
  const taskObj = getSelectedTaskObj();
  const stepSel = document.getElementById("deployStepSelect");
  if (!taskObj || !stepSel) return;
  const ckpts = taskObj.checkpoints || [];
  stepSel.innerHTML = ckpts
    .map((c) => `<option value="${c.step}" data-path="${c.path}" data-weights="${c.has_weights}">step ${c.step} ${c.has_weights ? "✓" : "(未见权重)"}</option>`)
    .join("");
  const targetStep = preferredStep || taskObj.default_step;
  if (targetStep && ckpts.some((c) => String(c.step) === String(targetStep))) {
    stepSel.value = String(targetStep);
  }
  const promptInput = document.getElementById("deployPromptInput");
  if (promptInput && taskObj.prompt) {
    promptInput.value = taskObj.prompt;
  }
  updateSelectedTaskMeta();
}

function onDeployStepChange() {
  updateSelectedTaskMeta();
}

function updateSelectedTaskMeta() {
  const taskObj = getSelectedTaskObj();
  const stepSel = document.getElementById("deployStepSelect");
  if (!taskObj || !stepSel) return;

  const ckptObj = (taskObj.checkpoints || []).find((c) => String(c.step) === String(stepSel.value)) || (taskObj.checkpoints || [])[0];
  const ckptPathEl = document.getElementById("deployCkptPathDisplay");
  if (ckptPathEl) {
    ckptPathEl.textContent = ckptObj ? ckptObj.path : taskObj.checkpoint_base_dir;
  }

  const metaEl = document.getElementById("deployServerMetaDisplay");
  const gpuBadge = document.getElementById("gpuPreloadBadge");
  const isReady = taskObj.server_status === "ready";
  const isLoading = taskObj.server_status === "loading";

  if (metaEl) {
    metaEl.textContent = `端口: ${taskObj.port} | GPU: ${taskObj.gpu} | 状态: ${
      isReady ? "GPU 常驻就绪 (秒级启动)" : isLoading ? "正在加载进 GPU..." : "未加载"
    } | 权重: ${ckptObj?.has_weights ? "已验证存在" : "未找到"}`;
  }
  if (gpuBadge) {
    gpuBadge.textContent = isReady ? `● GPU ${taskObj.gpu} 已就绪 (:${taskObj.port})` : isLoading ? "⏳ GPU 预热中..." : "未预热";
    gpuBadge.className = "mini-pill " + (isReady ? "ok" : isLoading ? "warn" : "");
  }
  setDot("subDotGpu", isReady ? "ok" : isLoading ? "warn" : "");
}

async function addCustomDeployRoot() {
  const val = document.getElementById("customDeployRootInput")?.value?.trim();
  if (!val) return;
  await fetch("/api/config/update", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ custom_deploy_root: val }),
  });
  document.getElementById("customDeployRootInput").value = "";
  refreshState();
}

async function preloadDeployToGpu() {
  const depObj = getSelectedDeployObj();
  const taskObj = getSelectedTaskObj();
  const step = document.getElementById("deployStepSelect")?.value || "2000";
  if (!depObj || !taskObj) return;
  setLastOp("PRELOAD", `预加载 ${taskObj.task_id} (step ${step}) 进 GPU...`);
  const res = await fetch("/api/deploy/preload", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      deploy_root: depObj.deploy_root,
      task_id: taskObj.task_id,
      checkpoint_step: step,
    }),
  });
  const data = await res.json();
  setLastOp(data.ok ? "PRELOAD" : "WARN", data.message || "预加载请求已发送");
  refreshState();
}

async function unloadDeployFromGpu() {
  const depObj = getSelectedDeployObj();
  const taskObj = getSelectedTaskObj();
  if (!depObj || !taskObj) return;
  const res = await fetch("/api/deploy/unload", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      deploy_root: depObj.deploy_root,
      task_id: taskObj.task_id,
    }),
  });
  const data = await res.json();
  setLastOp("UNLOAD", data.message || "已卸载 GPU 模型");
  refreshState();
}

async function startDeployRollout() {
  const depObj = getSelectedDeployObj();
  const taskObj = getSelectedTaskObj();
  const step = document.getElementById("deployStepSelect")?.value || "2000";
  if (!depObj || !taskObj) return;

  const ckptPath = document.getElementById("deployCkptPathDisplay")?.textContent || "";
  const payload = {
    deploy_root: depObj.deploy_root,
    task_id: taskObj.task_id,
    checkpoint_step: String(step),
    checkpoint_path: ckptPath,
    prompt: document.getElementById("deployPromptInput")?.value || taskObj.prompt,
    launch_mode: document.getElementById("deployLaunchMode")?.value || "hil_fast",
    enable_hil: document.getElementById("hilEnableSelect")?.value === "true",
    auto_resume: document.getElementById("hilAutoResumeSelect")?.value === "true",
    idle_resume_sec: parseFloat(document.getElementById("hilIdleSecInput")?.value || "1.2"),
    spacemouse_deadzone: parseFloat(document.getElementById("hilDeadzoneInput")?.value || "0.15"),
    record_mode: document.getElementById("deployRecordMode")?.value || "hil_only",
    record_dir: document.getElementById("deployRecordDir")?.value || "",
    skip_reset: document.getElementById("deploySkipReset")?.value === "true",
    cam_width: taskObj.cam_width || 424,
    cam_height: taskObj.cam_height || 240,
    fps: taskObj.cam_fps || 30,
  };

  setLastOp("DEPLOY", `启动部署 ${taskObj.task_id}...`);
  const res = await fetch("/api/deploy/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await res.json();
  if (data.need_preload) {
    if (confirm(`${data.message}\n\n是否现在立即为您启动 GPU 预加载？`)) {
      preloadDeployToGpu();
    }
    return;
  }
  setLastOp(data.ok ? "DEPLOY" : "WARN", data.message || "部署请求已返回");
  refreshState();
}

async function sendDeployControl(payload) {
  const res = await fetch("/api/deploy/control", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await res.json();
  setLastOp("DEPLOY", data.message || payload.action);
  refreshState();
}

function pushLiveHilConfig() {
  sendDeployControl({
    action: "update_hil_config",
    enable_hil: document.getElementById("hilEnableSelect")?.value === "true",
    auto_resume: document.getElementById("hilAutoResumeSelect")?.value === "true",
    idle_resume_sec: parseFloat(document.getElementById("hilIdleSecInput")?.value || "1.2"),
  });
}

function renderRolloutLiveStatus(activeProc, st, cam) {
  const badge = document.getElementById("rolloutModeBadge");
  const banner = document.getElementById("hilLiveBanner");
  const tag = document.getElementById("hilModeTag");
  const txt = document.getElementById("hilBannerText");
  const stats = document.getElementById("hilBannerStats");

  const mode = activeProc ? st.mode || "POLICY" : "IDLE";
  if (badge) {
    badge.textContent = activeProc ? `${mode} (PID ${activeProc.pid})` : "IDLE";
    badge.className = "mini-pill " + (mode === "POLICY" ? "ok" : mode === "HIL" || mode === "PAUSED" ? "warn" : "");
  }
  if (tag) tag.textContent = mode;
  if (banner) {
    banner.className =
      "hil-live-banner " + (mode === "HIL" || mode === "PAUSED" ? "hil-active" : mode === "POLICY" ? "policy-active" : "");
  }
  if (txt) {
    if (!activeProc) {
      txt.textContent = "当前未运行部署推理（快捷键 Alt + P 可随时一键暂停 / 继续推理）";
    } else if (mode === "HIL") {
      txt.textContent = "✋ SpaceMouse HIL 人工接管中！策略推理已暂停并正在自动录制 HIL 轨迹...";
    } else if (mode === "PAUSED") {
      txt.textContent = "⏸ 推理已暂停（可移动 SpaceMouse 微调，或按 Alt+P / 点击恢复推理继续）";
    } else {
      txt.textContent = "▶ 策略推理运行中（推动 SpaceMouse 可随时自动打断并切入人工 HIL 接管）";
    }
  }
  if (stats) {
    const hz = typeof (st.capture_hz ?? cam?.capture_hz) === "number" ? (st.capture_hz ?? cam.capture_hz).toFixed(1) : "--";
    stats.textContent = `Step: ${st.step || 0} | 录制帧: ${st.recorded_frames || 0} | HIL 帧: ${st.hil_frames || 0} | 采集频率: ${hz} Hz`;
  }
}

// ==================== 6. GPU 监控、日志与设置弹窗 ====================
function renderGpuCards(gpus) {
  const grid = document.getElementById("gpuCardsGrid");
  if (!grid) return;
  if (!Array.isArray(gpus) || gpus.length === 0) {
    grid.innerHTML = `<div class="muted">未探测到 NVIDIA GPU</div>`;
    return;
  }
  grid.innerHTML = gpus
    .map((g) => {
      const pct = g.memory_total_mb > 0 ? Math.round((g.memory_used_mb / g.memory_total_mb) * 100) : 0;
      return `
      <div class="gpu-card">
        <div style="display:flex;justify-content:space-between;font-weight:700;">
          <span>GPU ${g.index}: ${g.name}</span>
          <span class="mono">${g.utilization_pct}%</span>
        </div>
        <div class="muted mono" style="margin-top:4px;">显存: ${g.memory_used_mb} / ${g.memory_total_mb} MB (${pct}%)</div>
        <div class="gpu-bar-bg"><div class="gpu-bar-fill" style="width:${pct}%"></div></div>
      </div>
    `;
    })
    .join("");
}

function renderLogs(logs) {
  const pre = document.getElementById("logConsole");
  if (!pre) return;
  const visible = logs.slice(clearedLogCount);
  pre.textContent = visible.length > 0 ? visible.join("\n") : "暂无新日志输出...";
  const autoScroll = document.getElementById("chkAutoScroll")?.checked;
  if (autoScroll) {
    pre.scrollTop = pre.scrollHeight;
  }
}

function clearConsoleView() {
  if (globalState && Array.isArray(globalState.logs)) {
    clearedLogCount = globalState.logs.length;
  }
  const pre = document.getElementById("logConsole");
  if (pre) pre.textContent = "已清空当前视图日志。";
}

async function stopAllActiveTasks() {
  if (globalState?.record?.active) {
    await sendRecordControl("kill");
  }
  if (globalState?.deploy?.active_rollout) {
    await sendDeployControl({ action: "stop" });
  }
  setLastOp("STOP", "已发送停止当前任务指令");
}

function toggleSettingsModal(forceOpen) {
  const m = document.getElementById("settingsModal");
  if (!m) return;
  const show = typeof forceOpen === "boolean" ? forceOpen : m.style.display === "none";
  m.style.display = show ? "flex" : "none";
}

async function saveSettingsModal() {
  const condaEnv = document.getElementById("cfgCondaEnvInput")?.value?.trim();
  const storageRoot = document.getElementById("cfgStorageRootInput")?.value?.trim();
  await fetch("/api/config/update", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      conda_env: condaEnv || null,
      storage_root: storageRoot || null,
    }),
  });
  const deskUser = document.getElementById("deskUser")?.value?.trim();
  const deskPass = document.getElementById("deskPass")?.value?.trim();
  if (deskUser && deskPass) {
    await fetch("/api/device/desk", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        action: "status",
        username: deskUser,
        password: deskPass,
        save_credentials: true,
      }),
    });
  }
  toggleSettingsModal(false);
  setLastOp("CONFIG", "已保存环境与凭据设置");
  refreshState(true);
}

// 快捷键支持：Alt+P 暂停/继续推理；录制视图下方向键控制 Episode
window.addEventListener("keydown", (e) => {
  const tag = (e.target && e.target.tagName) || "";
  const inInput = tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT";

  if (e.altKey && (e.key === "p" || e.key === "P")) {
    e.preventDefault();
    sendDeployControl({ action: "toggle_pause" });
    return;
  }

  if (!inInput && globalState?.record?.active) {
    if (e.key === "ArrowRight") {
      e.preventDefault();
      sendRecordControl("save_episode");
    } else if (e.key === "ArrowLeft") {
      e.preventDefault();
      sendRecordControl("rerecord_episode");
    } else if (e.key === "Escape") {
      e.preventDefault();
      sendRecordControl("stop_recording");
    }
  }
});

window.addEventListener("DOMContentLoaded", () => {
  refreshState(true);
  pollCameraRealtime();
  setInterval(() => refreshState(false), 2000);
  setInterval(pollCameraRealtime, 180);
});
