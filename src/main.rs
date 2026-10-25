use anyhow::{Context, Result};
use axum::{
    extract::{Path as AxumPath, Query, State},
    http::header,
    response::{Html, IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::{
    collections::{HashMap, VecDeque},
    net::{SocketAddr, TcpStream},
    path::{Path, PathBuf},
    process::Stdio,
    sync::Arc,
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    process::Command,
    sync::Mutex,
};
use tower_http::cors::CorsLayer;

const MAX_LOG_LINES: usize = 600;

#[derive(Parser, Debug)]
#[command(name = "franka-web", version, about = "Franka Research 3 Web Console & CLI")]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,

    /// Listen host when running server
    #[arg(long, default_value = "0.0.0.0")]
    host: String,

    /// Listen port when running server
    #[arg(long, default_value_t = 8016)]
    port: u16,

    /// Franka-Teleop root path
    #[arg(long, default_value = "/data2/yjd/workspace/Franka-Teleop")]
    teleop_root: String,

    /// Default Conda environment path for Franka-Teleop commands
    #[arg(long, default_value = "/data2/yjd/.conda/envs/franka")]
    conda_env: String,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Start the HTTP web server (default)
    Serve {
        #[arg(long, default_value = "0.0.0.0")]
        host: String,
        #[arg(long, default_value_t = 8016)]
        port: u16,
    },
    /// Query current system & robot state from terminal (read-only)
    Status {
        #[arg(long, default_value = "http://127.0.0.1:8016")]
        server: String,
    },
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct AppConfig {
    pub project_root: PathBuf,
    pub runtime_dir: PathBuf,
    pub preview_dir: PathBuf,
    pub logs_dir: PathBuf,
    pub teleop_root: PathBuf,
    pub conda_env: PathBuf,
    pub storage_root: PathBuf,
    pub nuc_ip: String,
    pub nuc_port: u16,
    pub desk_port: u16,
    pub custom_deploy_roots: Vec<PathBuf>,
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct ManagedProcessInfo {
    pub name: String,
    pub pid: u32,
    pub cmd_summary: String,
    pub started_at: f64,
    pub log_file: String,
    pub extra: Value,
}

pub struct AppState {
    pub config: Mutex<AppConfig>,
    pub logs: Mutex<VecDeque<String>>,
    pub record_proc: Mutex<Option<ManagedProcessInfo>>,
    pub rollout_proc: Mutex<Option<ManagedProcessInfo>>,
    pub preload_procs: Mutex<HashMap<String, ManagedProcessInfo>>,
    pub cached_robot_state: Mutex<(f64, Value)>,
}

impl AppState {
    pub async fn append_log(&self, tag: &str, line: impl AsRef<str>) {
        let now = chrono::Local::now().format("%H:%M:%S");
        let formatted = format!("[{}] [{}] {}", now, tag, line.as_ref());
        let mut logs = self.logs.lock().await;
        if logs.len() >= MAX_LOG_LINES {
            logs.pop_front();
        }
        logs.push_back(formatted);
    }
}

fn now_sec() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

fn pid_alive(pid: u32) -> bool {
    let proc_path = PathBuf::from(format!("/proc/{}/stat", pid));
    if !proc_path.exists() {
        return false;
    }
    if let Ok(stat_str) = std::fs::read_to_string(&proc_path) {
        // Check if zombie: state character after "(comm) " is 'Z'
        if let Some(rparen) = stat_str.rfind(')') {
            let rest = stat_str[rparen + 1..].trim_start();
            if rest.starts_with('Z') {
                return false;
            }
        }
    }
    true
}

fn port_open_sync(host: &str, port: u16, timeout_ms: u64) -> bool {
    let addr_str = format!("{}:{}", host, port);
    if let Ok(addrs) = std::net::ToSocketAddrs::to_socket_addrs(&addr_str) {
        for addr in addrs {
            if TcpStream::connect_timeout(&addr, Duration::from_millis(timeout_ms)).is_ok() {
                return true;
            }
        }
    }
    false
}

async fn http_healthz_ok(port: u16) -> bool {
    let port_copy = port;
    tokio::task::spawn_blocking(move || {
        use std::io::{Read, Write};
        let addr: SocketAddr = format!("127.0.0.1:{}", port_copy).parse().ok()?;
        let mut stream = TcpStream::connect_timeout(&addr, Duration::from_millis(600)).ok()?;
        let _ = stream.set_read_timeout(Some(Duration::from_millis(800)));
        let _ = stream.write_all(b"GET /healthz HTTP/1.0\r\nHost: 127.0.0.1\r\n\r\n");
        let mut buf = [0u8; 128];
        let n = stream.read(&mut buf).ok()?;
        let head = String::from_utf8_lossy(&buf[..n]);
        Some(head.contains("200"))
    })
    .await
    .ok()
    .flatten()
    .unwrap_or(false)
}

fn find_pid_on_port(port: u16) -> Option<u32> {
    let out = std::process::Command::new("ss")
        .args(["-H", "-ltnp", &format!("sport = :{}", port)])
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    if let Some(idx) = text.find("pid=") {
        let rest = &text[idx + 4..];
        let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
        return digits.parse::<u32>().ok();
    }
    None
}

fn kill_process_group_or_pid(pid: u32) {
    #[cfg(unix)]
    unsafe {
        // Send SIGTERM to process group (-pid) and process (pid)
        libc_kill(-(pid as i32), 15);
        libc_kill(pid as i32, 15);
    }
}

#[cfg(unix)]
extern "C" {
    #[link_name = "kill"]
    fn libc_kill(pid: i32, sig: i32) -> i32;
}

fn discover_conda_envs(teleop_root: &Path) -> Vec<Value> {
    let mut envs = Vec::new();
    let conda_dirs = [
        PathBuf::from("/data2/yjd/.conda/envs"),
        PathBuf::from("/home/yjd/.conda/envs"),
        PathBuf::from("/home/LFT-W02/miniconda3/envs"),
        PathBuf::from("/home/LFT-W02/anaconda3/envs"),
    ];
    for base in conda_dirs {
        if let Ok(entries) = std::fs::read_dir(&base) {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.join("bin/python").exists() {
                    let has_franka_cli = p.join("bin/franka-record").exists();
                    envs.push(json!({
                        "name": format!("conda: {}", entry.file_name().to_string_lossy()),
                        "path": p.to_string_lossy(),
                        "has_franka_cli": has_franka_cli,
                    }));
                }
            }
        }
    }
    let venv_candidates = [
        teleop_root.join("junfeng/.venv"),
        PathBuf::from("/data2/yjd/workspace/openpi-franka-pytorch-deploy/.venv"),
    ];
    for vp in venv_candidates {
        if vp.join("bin/python").exists() {
            envs.push(json!({
                "name": format!("venv: {}", vp.display()),
                "path": vp.to_string_lossy(),
                "has_franka_cli": vp.join("bin/franka-record").exists(),
            }));
        }
    }
    if envs.is_empty() {
        envs.push(json!({
            "name": "default: /data2/yjd/.conda/envs/franka",
            "path": "/data2/yjd/.conda/envs/franka",
            "has_franka_cli": false,
        }));
    }
    envs
}

fn python_bin_for_env(conda_env: &Path) -> PathBuf {
    let candidate = conda_env.join("bin/python");
    if candidate.exists() {
        return candidate;
    }
    PathBuf::from("python3")
}

fn cli_bin_for_env(conda_env: &Path, bin_name: &str) -> PathBuf {
    let candidate = conda_env.join("bin").join(bin_name);
    if candidate.exists() {
        return candidate;
    }
    PathBuf::from(bin_name)
}

async fn spawn_logged_process(
    state: Arc<AppState>,
    tag: String,
    mut cmd: Command,
    log_path: PathBuf,
    stdin_input: Option<String>,
) -> Result<u32> {
    if let Some(parent) = log_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    cmd.stdout(Stdio::piped()).stderr(Stdio::piped());
    if stdin_input.is_some() {
        cmd.stdin(Stdio::piped());
    } else {
        cmd.stdin(Stdio::null());
    }
    #[cfg(unix)]
    unsafe {
        cmd.pre_exec(|| {
            // Create new session / process group so child tree can be cleanly signaled
            libc_setsid();
            Ok(())
        });
    }

    let mut child = cmd.spawn().context("Failed to spawn subprocess")?;
    let pid = child.id().unwrap_or(0);

    if let (Some(input_str), Some(mut stdin)) = (stdin_input, child.stdin.take()) {
        tokio::spawn(async move {
            let _ = stdin.write_all(input_str.as_bytes()).await;
            let _ = stdin.flush().await;
        });
    }

    let stdout = child.stdout.take();
    let stderr = child.stderr.take();
    let state_out = state.clone();
    let tag_out = tag.clone();
    let log_path_out = log_path.clone();

    if let Some(out) = stdout {
        tokio::spawn(async move {
            let mut reader = BufReader::new(out).lines();
            let mut file = tokio::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log_path_out)
                .await
                .ok();
            while let Ok(Some(line)) = reader.next_line().await {
                state_out.append_log(&tag_out, &line).await;
                if let Some(ref mut f) = file {
                    let _ = f.write_all(format!("{}\n", line).as_bytes()).await;
                }
            }
        });
    }

    let state_err = state.clone();
    let tag_err = tag.clone();
    let log_path_err = log_path;
    if let Some(err) = stderr {
        tokio::spawn(async move {
            let mut reader = BufReader::new(err).lines();
            let mut file = tokio::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log_path_err)
                .await
                .ok();
            while let Ok(Some(line)) = reader.next_line().await {
                state_err.append_log(&tag_err, &line).await;
                if let Some(ref mut f) = file {
                    let _ = f.write_all(format!("{}\n", line).as_bytes()).await;
                }
            }
        });
    }

    let state_wait = state;
    let tag_wait = tag;
    tokio::spawn(async move {
        if let Ok(status) = child.wait().await {
            state_wait
                .append_log(&tag_wait, format!("进程 (PID {}) 已退出: {}", pid, status))
                .await;
        }
    });

    Ok(pid)
}

#[cfg(unix)]
extern "C" {
    #[link_name = "setsid"]
    fn libc_setsid() -> i32;
}

async fn query_gpus() -> Vec<Value> {
    let out = Command::new("nvidia-smi")
        .args([
            "--query-gpu=index,name,memory.used,memory.total,utilization.gpu",
            "--format=csv,noheader,nounits",
        ])
        .output()
        .await;
    let Ok(out) = out else {
        return Vec::new();
    };
    let text = String::from_utf8_lossy(&out.stdout);
    let mut gpus = Vec::new();
    for line in text.lines() {
        let parts: Vec<&str> = line.split(',').map(|s| s.trim()).collect();
        if parts.len() >= 5 {
            gpus.push(json!({
                "index": parts[0],
                "name": parts[1],
                "memory_used_mb": parts[2].parse::<u64>().unwrap_or(0),
                "memory_total_mb": parts[3].parse::<u64>().unwrap_or(0),
                "utilization_pct": parts[4].parse::<u64>().unwrap_or(0),
            }));
        }
    }
    gpus
}

fn read_record_config_defaults(teleop_root: &Path) -> Value {
    let cfg_path = teleop_root.join("scripts/config/record_config.yaml");
    if let Ok(text) = std::fs::read_to_string(&cfg_path) {
        if let Ok(val) = serde_yaml::from_str::<Value>(&text) {
            let rec = &val["record"];
            return json!({
                "config_path": cfg_path.to_string_lossy(),
                "repo_id": rec["repo_id"].as_str().unwrap_or("hao/test"),
                "fps": rec["fps"].as_i64().unwrap_or(30),
                "description": rec["task"]["description"].as_str().unwrap_or(""),
                "num_episodes": rec["task"]["num_episodes"].as_i64().unwrap_or(5),
                "resume": rec["task"]["resume"].as_bool().unwrap_or(false),
                "resume_dataset": rec["task"]["resume_dataset"].as_str().unwrap_or(""),
                "robot_ip": rec["robot"]["ip"].as_str().unwrap_or("192.168.50.10"),
                "exterior_cam_id": rec["cameras"]["exterior_cam_id"].as_str().unwrap_or("CPC936300041"),
                "wrist_cam_id": rec["cameras"]["wrist_cam_id"].as_str().unwrap_or("CPC93630001C"),
                "width": rec["cameras"]["width"].as_i64().unwrap_or(424),
                "height": rec["cameras"]["height"].as_i64().unwrap_or(240),
            });
        }
    }
    json!({
        "config_path": cfg_path.to_string_lossy(),
        "repo_id": "hao/test",
        "fps": 30,
        "description": "Insert the green rectangular block from the table into the matching hole in the blue pegboard.",
        "num_episodes": 5,
        "resume": false,
        "resume_dataset": "",
        "robot_ip": "192.168.50.10",
        "exterior_cam_id": "CPC936300041",
        "wrist_cam_id": "CPC93630001C",
        "width": 424,
        "height": 240,
    })
}

fn known_task_port_gpu(task_id: &str) -> (u16, String) {
    match task_id {
        "carrot" => (8123, "1".to_string()),
        "kiwi" => (8124, "2".to_string()),
        "cups" => (8125, "3".to_string()),
        "pour_water" => (8126, "1".to_string()),
        "book_shelf" => (8127, "2".to_string()),
        "in_the_pot" => (8128, "3".to_string()),
        "green_pepper_plate_scene1" => (8129, "1".to_string()),
        "green_pepper_plate_scene2" => (8130, "2".to_string()),
        "green_block_insertion" => (8141, "3".to_string()),
        _ => (8145, "3".to_string()),
    }
}

pub async fn scan_deployment_roots(cfg: &AppConfig, preload_procs: &HashMap<String, ManagedProcessInfo>) -> Vec<Value> {
    let mut candidate_roots: Vec<PathBuf> = vec![
        cfg.teleop_root.join("xiqing/green_block_insertion"),
        cfg.teleop_root.join("junfeng"),
        cfg.teleop_root.join("deployments/green_block_insertion"),
        cfg.teleop_root.join("jiaan/green_block_insertion"),
    ];
    // Also discover any `<namespace>/<task>` under teleop_root that has `configs/` or `checkpoints/`
    if let Ok(entries) = std::fs::read_dir(&cfg.teleop_root) {
        for e in entries.flatten() {
            let p = e.path();
            if p.is_dir() {
                if p.join("configs").is_dir() || p.join("checkpoints").is_dir() {
                    if !candidate_roots.contains(&p) {
                        candidate_roots.push(p.clone());
                    }
                }
                if let Ok(sub_entries) = std::fs::read_dir(&p) {
                    for se in sub_entries.flatten() {
                        let sp = se.path();
                        if sp.is_dir() && (sp.join("configs").is_dir() || sp.join("checkpoints").is_dir()) {
                            if !candidate_roots.contains(&sp) {
                                candidate_roots.push(sp);
                            }
                        }
                    }
                }
            }
        }
    }
    for custom in &cfg.custom_deploy_roots {
        if !candidate_roots.contains(custom) {
            candidate_roots.push(custom.clone());
        }
    }

    let mut results = Vec::new();
    for root in candidate_roots {
        if !root.exists() {
            continue;
        }
        // Discover run_*.sh scripts
        let mut scripts = Vec::new();
        if let Ok(entries) = std::fs::read_dir(&root) {
            for e in entries.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if name.starts_with("run_") && name.ends_with(".sh") {
                    scripts.push(name);
                }
            }
        }
        scripts.sort();

        // Parse tasks from configs/tasks.yaml or configs/tasks.step_2000.yaml
        let mut tasks_out = Vec::new();
        let yaml_candidates = [
            root.join("configs/tasks.yaml"),
            root.join("configs/tasks.step_2000.yaml"),
            root.join("configs/tasks.step_4000.yaml"),
        ];
        let mut parsed_yaml: Option<Value> = None;
        for yc in &yaml_candidates {
            if let Ok(txt) = std::fs::read_to_string(yc) {
                if let Ok(v) = serde_yaml::from_str::<Value>(&txt) {
                    parsed_yaml = Some(v);
                    break;
                }
            }
        }

        let cam_width = parsed_yaml
            .as_ref()
            .and_then(|v| v["cameras"]["exterior"]["width"].as_i64())
            .unwrap_or(424);
        let cam_height = parsed_yaml
            .as_ref()
            .and_then(|v| v["cameras"]["exterior"]["height"].as_i64())
            .unwrap_or(240);
        let cam_fps = parsed_yaml
            .as_ref()
            .and_then(|v| v["cameras"]["fps"].as_i64())
            .unwrap_or(30);

        if let Some(ref yml) = parsed_yaml {
            if let Some(tasks_seq) = yml["tasks"].as_array() {
                for t in tasks_seq {
                    let task_id = t["id"].as_str().unwrap_or("unknown").to_string();
                    let config_name = t["config"].as_str().unwrap_or("").to_string();
                    let prompt = t["prompt"].as_str().unwrap_or("").to_string();
                    let default_step = t["checkpoint_step"].as_i64().unwrap_or(1499);
                    let (port, gpu) = known_task_port_gpu(&task_id);

                    // Discover available checkpoint steps on disk under checkpoints/<task_id>/
                    let ckpt_task_dir = root.join("checkpoints").join(&task_id);
                    let mut steps: Vec<Value> = Vec::new();
                    if let Ok(step_entries) = std::fs::read_dir(&ckpt_task_dir) {
                        for se in step_entries.flatten() {
                            let sp = se.path();
                            if sp.is_dir() {
                                let step_name = se.file_name().to_string_lossy().to_string();
                                let has_safetensors = sp.join("model.safetensors").exists();
                                let has_manifest = sp.join("manifest.json").exists();
                                let has_orbax = sp.join("params").exists() || sp.join("_CHECKPOINT_METADATA").exists();
                                steps.push(json!({
                                    "step": step_name,
                                    "path": sp.to_string_lossy(),
                                    "exists": true,
                                    "has_weights": has_safetensors || has_orbax || has_manifest,
                                }));
                            }
                        }
                    }
                    steps.sort_by(|a, b| a["step"].as_str().cmp(&b["step"].as_str()));
                    let default_ckpt_path = ckpt_task_dir.join(default_step.to_string());
                    if steps.is_empty() {
                        steps.push(json!({
                            "step": default_step.to_string(),
                            "path": default_ckpt_path.to_string_lossy(),
                            "exists": default_ckpt_path.exists(),
                            "has_weights": default_ckpt_path.exists(),
                        }));
                    }

                    // Check if GPU server is READY or LOADING
                    let ready = http_healthz_ok(port).await;
                    let port_pid = find_pid_on_port(port);
                    let key = format!("{}:{}", root.display(), task_id);
                    let loading_proc = preload_procs.get(&key).filter(|p| pid_alive(p.pid));

                    let server_status = if ready {
                        "ready"
                    } else if loading_proc.is_some() {
                        "loading"
                    } else {
                        "stopped"
                    };

                    tasks_out.push(json!({
                        "task_id": task_id,
                        "config_name": config_name,
                        "prompt": prompt,
                        "default_step": default_step.to_string(),
                        "checkpoint_base_dir": ckpt_task_dir.to_string_lossy(),
                        "checkpoints": steps,
                        "port": port,
                        "gpu": gpu,
                        "server_status": server_status,
                        "server_pid": port_pid.or_else(|| loading_proc.map(|p| p.pid)),
                        "cam_width": cam_width,
                        "cam_height": cam_height,
                        "cam_fps": cam_fps,
                    }));
                }
            }
        }

        let has_policy_server_sh = root.join("scripts/policy_server.sh").exists();
        let has_task_sh = root.join("scripts/task.sh").exists();

        results.push(json!({
            "deploy_root": root.to_string_lossy(),
            "name": root.strip_prefix(&cfg.teleop_root).unwrap_or(&root).to_string_lossy(),
            "scripts": scripts,
            "has_policy_server_sh": has_policy_server_sh,
            "has_task_sh": has_task_sh,
            "tasks": tasks_out,
        }));
    }

    results
}

pub fn scan_datasets_sync(storage_root: &Path) -> Vec<Value> {
    let mut datasets = Vec::new();
    if !storage_root.exists() {
        return datasets;
    }

    // Look 1 and 2 levels deep under storage_root (e.g. ~/.cache/huggingface/lerobot/<user>/<dataset_name>)
    let mut dirs_to_check = Vec::new();
    if let Ok(level1) = std::fs::read_dir(storage_root) {
        for e1 in level1.flatten() {
            let p1 = e1.path();
            if !p1.is_dir() {
                continue;
            }
            let name1 = e1.file_name().to_string_lossy().to_string();
            if name1.starts_with('_') || name1.starts_with('.') {
                continue;
            }
            if p1.join("meta/info.json").exists() || p1.join("trajectory.jsonl").exists() {
                dirs_to_check.push((name1.clone(), p1));
            } else if let Ok(level2) = std::fs::read_dir(&p1) {
                for e2 in level2.flatten() {
                    let p2 = e2.path();
                    let name2 = e2.file_name().to_string_lossy().to_string();
                    if p2.is_dir()
                        && !name2.starts_with('_')
                        && !name2.starts_with('.')
                        && !name2.contains("backup")
                    {
                        if p2.join("meta/info.json").exists()
                            || p2.join("data").is_dir()
                            || p2.join("trajectory.jsonl").exists()
                        {
                            dirs_to_check.push((format!("{}/{}", name1, name2), p2));
                        }
                    }
                }
            }
        }
    }

    for (repo_id, ds_path) in dirs_to_check {
        let info_path = ds_path.join("meta/info.json");
        let hil_meta_path = ds_path.join("meta.json");
        let mtime = std::fs::metadata(&ds_path)
            .and_then(|m| m.modified())
            .ok()
            .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
            .map(|d| d.as_secs())
            .unwrap_or(0);

        if info_path.is_file() {
            let info: Value = std::fs::read_to_string(&info_path)
                .ok()
                .and_then(|s| serde_json::from_str(&s).ok())
                .unwrap_or_else(|| json!({}));

            // Also read tasks.jsonl or episodes.jsonl if present
            let mut prompt = String::new();
            let tasks_jsonl = ds_path.join("meta/tasks.jsonl");
            if let Ok(txt) = std::fs::read_to_string(&tasks_jsonl) {
                if let Some(first_line) = txt.lines().next() {
                    if let Ok(v) = serde_json::from_str::<Value>(first_line) {
                        prompt = v["task"].as_str().unwrap_or("").to_string();
                    }
                }
            }

            datasets.push(json!({
                "kind": "lerobot",
                "repo_id": repo_id,
                "path": ds_path.to_string_lossy(),
                "total_episodes": info["total_episodes"].as_i64().unwrap_or(0),
                "total_frames": info["total_frames"].as_i64().unwrap_or(0),
                "fps": info["fps"].as_i64().unwrap_or(30),
                "robot_type": info["robot_type"].as_str().unwrap_or("franka"),
                "prompt": prompt,
                "modified_ts": mtime,
            }));
        } else if hil_meta_path.is_file() {
            let meta: Value = std::fs::read_to_string(&hil_meta_path)
                .ok()
                .and_then(|s| serde_json::from_str(&s).ok())
                .unwrap_or_else(|| json!({}));
            let line_count = std::fs::read_to_string(ds_path.join("trajectory.jsonl"))
                .map(|s| s.lines().count())
                .unwrap_or(0);
            datasets.push(json!({
                "kind": "hil_rollout",
                "repo_id": repo_id,
                "path": ds_path.to_string_lossy(),
                "total_episodes": 1,
                "total_frames": line_count,
                "fps": meta["fps"].as_i64().unwrap_or(30),
                "robot_type": "franka_hil",
                "prompt": meta["prompt"].as_str().unwrap_or(""),
                "record_mode": meta["record_mode"].as_str().unwrap_or("hil_only"),
                "modified_ts": mtime,
            }));
        }
    }

    datasets.sort_by(|a, b| b["modified_ts"].as_u64().cmp(&a["modified_ts"].as_u64()));
    datasets
}

fn find_latest_camera_images(cfg: &AppConfig) -> Value {
    let ext_preview = cfg.preview_dir.join("exterior.jpg");
    let wrist_preview = cfg.preview_dir.join("wrist.jpg");
    let meta_preview = cfg.preview_dir.join("meta.json");

    let mut best_ext: Option<PathBuf> = None;
    let mut best_wrist: Option<PathBuf> = None;
    let mut best_mtime: f64 = 0.0;
    let mut source = String::from("none");
    let mut message = String::from("暂无相机画面（点击『抓取相机快照』或在录制/部署时自动实时显示）");

    if ext_preview.is_file() && wrist_preview.is_file() {
        if let Ok(m) = std::fs::metadata(&ext_preview).and_then(|m| m.modified()) {
            if let Ok(d) = m.duration_since(UNIX_EPOCH) {
                best_mtime = d.as_secs_f64();
                best_ext = Some(ext_preview);
                best_wrist = Some(wrist_preview);
                source = String::from("preview");
                if let Ok(txt) = std::fs::read_to_string(&meta_preview) {
                    if let Ok(v) = serde_json::from_str::<Value>(&txt) {
                        if let Some(s) = v["source"].as_str() {
                            source = s.to_string();
                        }
                        if let Some(msg) = v["message"].as_str() {
                            message = msg.to_string();
                        }
                    }
                }
            }
        }
    }

    // Also check latest deployment `runs/<task>/<run_id>/images/*_{exterior,wrist}.jpg`
    let run_roots = [
        cfg.teleop_root.join("xiqing/green_block_insertion/runs"),
        cfg.teleop_root.join("junfeng/runs"),
    ];
    for rr in run_roots {
        if let Ok(task_dirs) = std::fs::read_dir(&rr) {
            for td in task_dirs.flatten() {
                if let Ok(run_dirs) = std::fs::read_dir(td.path()) {
                    let mut runs: Vec<PathBuf> = run_dirs.flatten().map(|r| r.path()).collect();
                    runs.sort();
                    if let Some(latest_run) = runs.last() {
                        let img_dir = latest_run.join("images");
                        if let Ok(imgs) = std::fs::read_dir(&img_dir) {
                            let mut ext_list = Vec::new();
                            let mut wrist_list = Vec::new();
                            for im in imgs.flatten() {
                                let name = im.file_name().to_string_lossy().to_string();
                                if name.ends_with("_exterior.jpg") {
                                    ext_list.push(im.path());
                                } else if name.ends_with("_wrist.jpg") {
                                    wrist_list.push(im.path());
                                }
                            }
                            ext_list.sort();
                            wrist_list.sort();
                            if let (Some(le), Some(lw)) = (ext_list.last(), wrist_list.last()) {
                                if let Ok(m) = std::fs::metadata(le).and_then(|m| m.modified()) {
                                    if let Ok(d) = m.duration_since(UNIX_EPOCH) {
                                        let mt = d.as_secs_f64();
                                        if mt > best_mtime {
                                            best_mtime = mt;
                                            best_ext = Some(le.clone());
                                            best_wrist = Some(lw.clone());
                                            source = format!("run:{}", latest_run.file_name().unwrap_or_default().to_string_lossy());
                                            message = format!("来自部署运行日志 ({})", source);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    let age_seconds = if best_mtime > 0.0 {
        (now_sec() - best_mtime).max(0.0)
    } else {
        -1.0
    };

    json!({
        "ready": best_ext.is_some() && best_wrist.is_some(),
        "age_seconds": age_seconds,
        "updated_at": best_mtime,
        "source": source,
        "message": message,
        "exterior_path": best_ext.map(|p| p.to_string_lossy().to_string()),
        "wrist_path": best_wrist.map(|p| p.to_string_lossy().to_string()),
    })
}

async fn get_robot_state_cached(state: &Arc<AppState>, force_refresh: bool) -> Value {
    let now = now_sec();
    {
        let cached = state.cached_robot_state.lock().await;
        if !force_refresh && cached.0 > 0.0 && (now - cached.0) < 2.5 {
            return cached.1.clone();
        }
    }

    let cfg = state.config.lock().await.clone();
    let py = python_bin_for_env(&cfg.conda_env);
    let helper = cfg.project_root.join("scripts/franka_helper.py");

    let out = tokio::time::timeout(
        Duration::from_secs(5),
        Command::new(&py)
            .arg(&helper)
            .arg("robot-state")
            .arg("--teleop-root")
            .arg(&cfg.teleop_root)
            .arg("--nuc-ip")
            .arg(&cfg.nuc_ip)
            .arg("--nuc-port")
            .arg(cfg.nuc_port.to_string())
            .arg("--desk-port")
            .arg(cfg.desk_port.to_string())
            .output(),
    )
    .await;

    let val = match out {
        Ok(Ok(output)) => {
            let stdout = String::from_utf8_lossy(&output.stdout);
            serde_json::from_str::<Value>(stdout.trim()).unwrap_or_else(|_| {
                json!({
                    "nuc_ip": cfg.nuc_ip,
                    "nuc_port": cfg.nuc_port,
                    "nuc_online": port_open_sync(&cfg.nuc_ip, cfg.nuc_port, 500),
                    "desk_tunnel_online": port_open_sync("127.0.0.1", cfg.desk_port, 300),
                    "desk_url": format!("https://localhost:{}/desk/", cfg.desk_port),
                    "robot_readable": false,
                    "error": String::from_utf8_lossy(&output.stderr).trim().to_string(),
                })
            })
        }
        _ => json!({
            "nuc_ip": cfg.nuc_ip,
            "nuc_port": cfg.nuc_port,
            "nuc_online": port_open_sync(&cfg.nuc_ip, cfg.nuc_port, 500),
            "desk_tunnel_online": port_open_sync("127.0.0.1", cfg.desk_port, 300),
            "desk_url": format!("https://localhost:{}/desk/", cfg.desk_port),
            "robot_readable": false,
            "error": "状态探测超时或环境未就绪",
        }),
    };

    let mut cached = state.cached_robot_state.lock().await;
    *cached = (now_sec(), val.clone());
    val
}

// ==================== HTTP Handlers ====================

async fn index_handler(State(state): State<Arc<AppState>>) -> Response {
    let cfg = state.config.lock().await.clone();
    let path = cfg.project_root.join("frontend/index.html");
    let content = tokio::fs::read_to_string(&path)
        .await
        .unwrap_or_else(|_| include_str!("../frontend/index.html").to_string());
    Html(content).into_response()
}

async fn css_handler(State(state): State<Arc<AppState>>) -> Response {
    let cfg = state.config.lock().await.clone();
    let path = cfg.project_root.join("frontend/styles.css");
    let content = tokio::fs::read_to_string(&path)
        .await
        .unwrap_or_else(|_| include_str!("../frontend/styles.css").to_string());
    ([(header::CONTENT_TYPE, "text/css; charset=utf-8")], content).into_response()
}

async fn js_handler(State(state): State<Arc<AppState>>) -> Response {
    let cfg = state.config.lock().await.clone();
    let path = cfg.project_root.join("frontend/app.js");
    let content = tokio::fs::read_to_string(&path)
        .await
        .unwrap_or_else(|_| include_str!("../frontend/app.js").to_string());
    (
        [(header::CONTENT_TYPE, "application/javascript; charset=utf-8")],
        content,
    )
        .into_response()
}

async fn api_state_handler(
    State(state): State<Arc<AppState>>,
    Query(params): Query<HashMap<String, String>>,
) -> Json<Value> {
    let force_robot = params.get("refresh_robot").map(|v| v == "1").unwrap_or(false);
    let cfg = state.config.lock().await.clone();

    // Clean up exited record/rollout processes
    {
        let mut rec = state.record_proc.lock().await;
        if let Some(ref p) = *rec {
            if !pid_alive(p.pid) {
                *rec = None;
            }
        }
    }
    {
        let mut rol = state.rollout_proc.lock().await;
        if let Some(ref p) = *rol {
            if !pid_alive(p.pid) {
                *rol = None;
            }
        }
    }

    let robot = get_robot_state_cached(&state, force_robot).await;
    let gpus = query_gpus().await;
    let record_defaults = read_record_config_defaults(&cfg.teleop_root);
    let record_active = state.record_proc.lock().await.clone();
    let rollout_active = state.rollout_proc.lock().await.clone();
    let preload_map = state.preload_procs.lock().await.clone();
    let deployments = scan_deployment_roots(&cfg, &preload_map).await;
    let camera = find_latest_camera_images(&cfg);
    let conda_envs = discover_conda_envs(&cfg.teleop_root);

    // Read live rollout status file if active
    let rollout_status_file = cfg.runtime_dir.join("deploy_status.json");
    let rollout_live_detail: Value = if rollout_status_file.is_file() {
        std::fs::read_to_string(&rollout_status_file)
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_else(|| json!({}))
    } else {
        json!({})
    };

    let logs: Vec<String> = state.logs.lock().await.iter().cloned().collect();

    Json(json!({
        "ok": true,
        "timestamp": now_sec(),
        "config": {
            "teleop_root": cfg.teleop_root.to_string_lossy(),
            "conda_env": cfg.conda_env.to_string_lossy(),
            "storage_root": cfg.storage_root.to_string_lossy(),
            "nuc_ip": cfg.nuc_ip,
            "nuc_port": cfg.nuc_port,
            "desk_port": cfg.desk_port,
            "available_conda_envs": conda_envs,
        },
        "robot": robot,
        "gpus": gpus,
        "camera": camera,
        "record": {
            "active": record_active,
            "defaults": record_defaults,
        },
        "deploy": {
            "deployments": deployments,
            "active_rollout": rollout_active,
            "rollout_status": rollout_live_detail,
        },
        "logs": logs,
    }))
}

#[derive(Deserialize)]
struct UpdateConfigReq {
    conda_env: Option<String>,
    teleop_root: Option<String>,
    storage_root: Option<String>,
    nuc_ip: Option<String>,
    custom_deploy_root: Option<String>,
}

async fn api_update_config_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<UpdateConfigReq>,
) -> Json<Value> {
    let mut cfg = state.config.lock().await;
    if let Some(env) = req.conda_env {
        if !env.trim().is_empty() {
            cfg.conda_env = PathBuf::from(env.trim());
        }
    }
    if let Some(tr) = req.teleop_root {
        if !tr.trim().is_empty() {
            cfg.teleop_root = PathBuf::from(tr.trim());
        }
    }
    if let Some(sr) = req.storage_root {
        if !sr.trim().is_empty() {
            cfg.storage_root = PathBuf::from(sr.trim());
        }
    }
    if let Some(ip) = req.nuc_ip {
        if !ip.trim().is_empty() {
            cfg.nuc_ip = ip.trim().to_string();
        }
    }
    if let Some(cdr) = req.custom_deploy_root {
        let p = PathBuf::from(cdr.trim());
        if !cdr.trim().is_empty() && !cfg.custom_deploy_roots.contains(&p) {
            cfg.custom_deploy_roots.push(p);
        }
    }
    drop(cfg);
    state.append_log("CONFIG", "已更新运行环境与路径配置").await;
    Json(json!({"ok": true, "message": "配置已更新"}))
}

#[derive(Deserialize)]
struct DeviceCommandReq {
    command: String,
}

async fn api_device_command_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<DeviceCommandReq>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let cmd_name = req.command.as_str();

    let (bin_or_py, args, stdin_data, label): (PathBuf, Vec<String>, Option<String>, String) = match cmd_name {
        "connect" => (
            cli_bin_for_env(&cfg.conda_env, "franka-connect"),
            vec![],
            None,
            "franka-connect (开启 Desk 隧道)".to_string(),
        ),
        "disconnect" => (
            cli_bin_for_env(&cfg.conda_env, "franka-disconnect"),
            vec![],
            None,
            "franka-disconnect (关闭 Desk 隧道)".to_string(),
        ),
        "start" => (
            cli_bin_for_env(&cfg.conda_env, "franka-start"),
            vec![],
            None,
            "franka-start (启动 NUC Polymetis + zerorpc 服务)".to_string(),
        ),
        "stop" => (
            cli_bin_for_env(&cfg.conda_env, "franka-stop"),
            vec![],
            None,
            "franka-stop (停止 NUC 服务)".to_string(),
        ),
        "status" => (
            cli_bin_for_env(&cfg.conda_env, "franka-status"),
            vec![],
            None,
            "franka-status (查询 NUC 服务状态)".to_string(),
        ),
        "gripper_reinit" => (
            cli_bin_for_env(&cfg.conda_env, "franka-gripper-reinit"),
            vec![],
            None,
            "franka-gripper-reinit (重启 NUC 夹爪驱动)".to_string(),
        ),
        "reset" => (
            cli_bin_for_env(&cfg.conda_env, "franka-reset"),
            vec![],
            Some("y\n".to_string()),
            "franka-reset (机械臂回 Home 位并打开夹爪)".to_string(),
        ),
        "preflight" => (
            cli_bin_for_env(&cfg.conda_env, "franka-record"),
            vec!["--check".to_string()],
            None,
            "franka-record --check (采集前五项预检)".to_string(),
        ),
        "camera_snapshot" => {
            if state.record_proc.lock().await.is_some() || state.rollout_proc.lock().await.is_some() {
                return Json(json!({
                    "ok": false,
                    "message": "当前正在进行录制或部署，相机画面会自动实时更新，无需手动抓拍以免抢占相机句柄。"
                }));
            }
            (
                python_bin_for_env(&cfg.conda_env),
                vec![
                    cfg.project_root.join("scripts/franka_helper.py").to_string_lossy().to_string(),
                    "camera-snapshot".to_string(),
                    "--teleop-root".to_string(),
                    cfg.teleop_root.to_string_lossy().to_string(),
                    "--preview-dir".to_string(),
                    cfg.preview_dir.to_string_lossy().to_string(),
                ],
                None,
                "抓取双 Orbbec 相机快照".to_string(),
            )
        }
        _ => {
            return Json(json!({"ok": false, "message": format!("未知设备命令: {}", cmd_name)}));
        }
    };

    state.append_log("DEVICE", format!("执行: {}", label)).await;

    let mut command = Command::new(&bin_or_py);
    command.args(&args).current_dir(&cfg.teleop_root);
    let bin_dir = cfg.conda_env.join("bin");
    if let Ok(old_path) = std::env::var("PATH") {
        command.env("PATH", format!("{}:{}", bin_dir.display(), old_path));
    }

    if stdin_data.is_some() {
        command.stdin(Stdio::piped());
    } else {
        command.stdin(Stdio::null());
    }
    command.stdout(Stdio::piped()).stderr(Stdio::piped());

    let mut child = match command.spawn() {
        Ok(c) => c,
        Err(e) => {
            let msg = format!("启动 `{}` 失败: {}", label, e);
            state.append_log("DEVICE", &msg).await;
            return Json(json!({"ok": false, "message": msg}));
        }
    };

    if let (Some(input_s), Some(mut stdin)) = (stdin_data, child.stdin.take()) {
        let _ = stdin.write_all(input_s.as_bytes()).await;
        let _ = stdin.flush().await;
    }

    let wait_res = tokio::time::timeout(Duration::from_secs(35), child.wait_with_output()).await;
    match wait_res {
        Ok(Ok(output)) => {
            let out_str = String::from_utf8_lossy(&output.stdout).trim().to_string();
            let err_str = String::from_utf8_lossy(&output.stderr).trim().to_string();
            for line in out_str.lines().chain(err_str.lines()) {
                if !line.trim().is_empty() {
                    state.append_log("DEVICE", line).await;
                }
            }
            let ok = output.status.success();
            let _ = get_robot_state_cached(&state, true).await;
            Json(json!({
                "ok": ok,
                "command": cmd_name,
                "exit_code": output.status.code(),
                "stdout": out_str,
                "stderr": err_str,
                "message": if ok { format!("{} 执行完成", label) } else { format!("{} 退出码非零", label) },
            }))
        }
        _ => {
            state.append_log("DEVICE", format!("{} 执行超时 (35s)", label)).await;
            Json(json!({"ok": false, "message": format!("{} 执行超时", label)}))
        }
    }
}

#[derive(Deserialize)]
struct DeskActionReq {
    action: String,
    username: Option<String>,
    password: Option<String>,
    save_credentials: Option<bool>,
}

async fn api_desk_action_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<DeskActionReq>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let py = python_bin_for_env(&cfg.conda_env);
    let helper = cfg.project_root.join("scripts/franka_helper.py");

    state
        .append_log("DESK", format!("请求 Franka Desk 操作: {}", req.action))
        .await;

    let mut cmd = Command::new(&py);
    cmd.arg(&helper)
        .arg("desk-action")
        .arg("--action")
        .arg(&req.action)
        .arg("--desk-port")
        .arg(cfg.desk_port.to_string())
        .arg("--runtime-dir")
        .arg(&cfg.runtime_dir);

    if let Some(ref u) = req.username {
        cmd.arg("--username").arg(u);
    }
    if let Some(ref p) = req.password {
        cmd.arg("--password").arg(p);
    }
    if req.save_credentials.unwrap_or(false) {
        cmd.arg("--save-credentials");
    }

    match tokio::time::timeout(Duration::from_secs(40), cmd.output()).await {
        Ok(Ok(out)) => {
            let s = String::from_utf8_lossy(&out.stdout);
            if let Ok(v) = serde_json::from_str::<Value>(s.trim()) {
                if let Some(msg) = v["message"].as_str() {
                    state.append_log("DESK", msg).await;
                }
                let _ = get_robot_state_cached(&state, true).await;
                return Json(v);
            }
            Json(json!({"ok": out.status.success(), "message": s.trim()}))
        }
        _ => Json(json!({"ok": false, "message": "Franka Desk 操作超时"})),
    }
}

#[derive(Deserialize)]
struct StartRecordReq {
    repo_id: String,
    storage_root: Option<String>,
    description: String,
    num_episodes: u32,
    fps: u32,
    resume: Option<bool>,
    resume_dataset: Option<String>,
}

async fn api_record_start_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<StartRecordReq>,
) -> Json<Value> {
    {
        let rec = state.record_proc.lock().await;
        if let Some(ref p) = *rec {
            if pid_alive(p.pid) {
                return Json(json!({"ok": false, "message": format!("录制已在运行中 (PID {})", p.pid)}));
            }
        }
    }
    {
        let rol = state.rollout_proc.lock().await;
        if let Some(ref p) = *rol {
            if pid_alive(p.pid) {
                return Json(json!({"ok": false, "message": format!("当前有正在运行的部署任务 (PID {})，请先停止部署再开始录制。", p.pid)}));
            }
        }
    }

    let cfg = state.config.lock().await.clone();
    let base_cfg_path = cfg.teleop_root.join("scripts/config/record_config.yaml");
    let mut yaml_doc: Value = std::fs::read_to_string(&base_cfg_path)
        .ok()
        .and_then(|s| serde_yaml::from_str(&s).ok())
        .unwrap_or_else(|| json!({"record": {}}));

    let storage_root = req
        .storage_root
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| cfg.storage_root.to_string_lossy().to_string());

    yaml_doc["record"]["repo_id"] = json!(req.repo_id.trim());
    yaml_doc["record"]["fps"] = json!(req.fps);
    yaml_doc["record"]["task"]["description"] = json!(req.description.trim());
    yaml_doc["record"]["task"]["num_episodes"] = json!(req.num_episodes);
    yaml_doc["record"]["task"]["resume"] = json!(req.resume.unwrap_or(false));
    if let Some(ref rds) = req.resume_dataset {
        if !rds.trim().is_empty() {
            yaml_doc["record"]["task"]["resume_dataset"] = json!(rds.trim());
        }
    }

    let active_cfg_path = cfg.runtime_dir.join("record_config.active.yaml");
    if let Ok(yaml_str) = serde_yaml::to_string(&yaml_doc) {
        let _ = std::fs::write(&active_cfg_path, yaml_str);
    }

    let control_file = cfg.runtime_dir.join("record_cmd.json");
    let _ = std::fs::remove_file(&control_file);
    let log_file = cfg.logs_dir.join("record.log");

    let py = python_bin_for_env(&cfg.conda_env);
    let helper = cfg.project_root.join("scripts/franka_helper.py");

    let mut cmd = Command::new(&py);
    cmd.arg("-u")
        .arg(&helper)
        .arg("record-wrapper")
        .arg("--teleop-root")
        .arg(&cfg.teleop_root)
        .arg("--config-file")
        .arg(&active_cfg_path)
        .arg("--storage-root")
        .arg(&storage_root)
        .arg("--episodes")
        .arg(req.num_episodes.to_string())
        .arg("--preview-dir")
        .arg(&cfg.preview_dir)
        .arg("--control-file")
        .arg(&control_file)
        .current_dir(&cfg.teleop_root);

    let bin_dir = cfg.conda_env.join("bin");
    if let Ok(old_path) = std::env::var("PATH") {
        cmd.env("PATH", format!("{}:{}", bin_dir.display(), old_path));
    }

    match spawn_logged_process(state.clone(), "RECORD".to_string(), cmd, log_file.clone(), None).await {
        Ok(pid) => {
            let info = ManagedProcessInfo {
                name: format!("franka-record ({})", req.repo_id),
                pid,
                cmd_summary: format!("franka-record repo_id={} episodes={}", req.repo_id, req.num_episodes),
                started_at: now_sec(),
                log_file: log_file.to_string_lossy().to_string(),
                extra: json!({
                    "repo_id": req.repo_id,
                    "storage_root": storage_root,
                    "description": req.description,
                    "num_episodes": req.num_episodes,
                    "fps": req.fps,
                }),
            };
            *state.record_proc.lock().await = Some(info.clone());
            state
                .append_log(
                    "RECORD",
                    format!(
                        "已启动遥操作录制 (PID {}): repo_id={}, 存储根={}",
                        pid, req.repo_id, storage_root
                    ),
                )
                .await;
            Json(json!({"ok": true, "pid": pid, "info": info, "message": "录制已启动"}))
        }
        Err(e) => Json(json!({"ok": false, "message": format!("启动录制失败: {}", e)})),
    }
}

#[derive(Deserialize)]
struct RecordControlReq {
    action: String,
}

async fn api_record_control_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<RecordControlReq>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    if req.action == "kill" {
        let mut rec = state.record_proc.lock().await;
        if let Some(p) = rec.take() {
            kill_process_group_or_pid(p.pid);
            state
                .append_log("RECORD", format!("已发送停止信号给录制进程 PID {}", p.pid))
                .await;
            return Json(json!({"ok": true, "message": "已强制停止录制进程"}));
        }
        return Json(json!({"ok": false, "message": "当前没有运行中的录制进程"}));
    }

    let control_file = cfg.runtime_dir.join("record_cmd.json");
    let _ = std::fs::write(&control_file, json!({"action": req.action, "ts": now_sec()}).to_string());
    state
        .append_log("RECORD", format!("已发送网页录制指令: {}", req.action))
        .await;
    Json(json!({"ok": true, "message": format!("已发送指令: {}", req.action)}))
}

async fn api_datasets_list_handler(
    State(state): State<Arc<AppState>>,
    Query(params): Query<HashMap<String, String>>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let root = params
        .get("storage_root")
        .filter(|s| !s.trim().is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| cfg.storage_root.clone());

    let datasets = tokio::task::spawn_blocking(move || scan_datasets_sync(&root))
        .await
        .unwrap_or_default();

    Json(json!({
        "ok": true,
        "storage_root": cfg.storage_root.to_string_lossy(),
        "datasets": datasets,
    }))
}

#[derive(Deserialize)]
struct DatasetPreviewReq {
    dataset_dir: String,
    episode_index: Option<u32>,
}

async fn api_dataset_preview_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<DatasetPreviewReq>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let py = python_bin_for_env(&cfg.conda_env);
    let helper = cfg.project_root.join("scripts/franka_helper.py");
    let ep_idx = req.episode_index.unwrap_or(0);

    state
        .append_log(
            "DATASET",
            format!("加载已有录制数据集预览: {} (episode {})", req.dataset_dir, ep_idx),
        )
        .await;

    let out = Command::new(&py)
        .arg(&helper)
        .arg("dataset-preview")
        .arg("--dataset-dir")
        .arg(&req.dataset_dir)
        .arg("--episode-index")
        .arg(ep_idx.to_string())
        .arg("--preview-dir")
        .arg(&cfg.preview_dir)
        .output()
        .await;

    match out {
        Ok(output) => {
            let s = String::from_utf8_lossy(&output.stdout);
            if let Ok(v) = serde_json::from_str::<Value>(s.trim()) {
                return Json(v);
            }
            Json(json!({
                "ok": output.status.success(),
                "stdout": s.trim(),
                "stderr": String::from_utf8_lossy(&output.stderr).trim(),
            }))
        }
        Err(e) => Json(json!({"ok": false, "error": e.to_string()})),
    }
}

#[derive(Deserialize)]
struct DatasetReplayReq {
    dataset_name: String,
    episode_index: u32,
    check_only: Option<bool>,
}

async fn api_dataset_replay_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<DatasetReplayReq>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let check_only = req.check_only.unwrap_or(false);

    // Update replay section in record_config.yaml or active copy
    let base_cfg_path = cfg.teleop_root.join("scripts/config/record_config.yaml");
    if let Ok(txt) = std::fs::read_to_string(&base_cfg_path) {
        if let Ok(mut yml) = serde_yaml::from_str::<Value>(&txt) {
            yml["replay"]["dataset_name"] = json!(req.dataset_name);
            yml["replay"]["episode_idx"] = json!(req.episode_index);
            if let Ok(new_txt) = serde_yaml::to_string(&yml) {
                let _ = std::fs::write(&base_cfg_path, new_txt);
            }
        }
    }

    let replay_bin = cli_bin_for_env(&cfg.conda_env, "franka-replay");
    let mut cmd = Command::new(&replay_bin);
    if check_only {
        cmd.args(["--last", "--check"]);
    }
    cmd.current_dir(&cfg.teleop_root);
    let log_file = cfg.logs_dir.join("replay.log");
    let stdin_data = if check_only {
        None
    } else {
        Some("REPLAY\n".to_string())
    };

    match spawn_logged_process(
        state.clone(),
        "REPLAY".to_string(),
        cmd,
        log_file,
        stdin_data,
    )
    .await
    {
        Ok(pid) => Json(json!({
            "ok": true,
            "pid": pid,
            "message": format!("已启动回放任务 {}#ep{} (PID {})", req.dataset_name, req.episode_index, pid)
        })),
        Err(e) => Json(json!({"ok": false, "message": format!("启动回放失败: {}", e)})),
    }
}

#[derive(Deserialize)]
struct PreloadDeployReq {
    deploy_root: String,
    task_id: String,
    checkpoint_step: Option<String>,
}

async fn api_deploy_preload_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<PreloadDeployReq>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let root = PathBuf::from(req.deploy_root.trim());
    if !root.exists() {
        return Json(json!({"ok": false, "message": format!("部署目录不存在: {}", root.display())}));
    }

    let (port, gpu) = known_task_port_gpu(&req.task_id);
    if http_healthz_ok(port).await {
        return Json(json!({
            "ok": true,
            "already_ready": true,
            "port": port,
            "gpu": gpu,
            "message": format!("任务 {} 的 Policy Server 已在端口 {} (GPU {}) 就绪，可直接点击『开始部署』秒级启动！", req.task_id, port, gpu),
        }));
    }

    let step = req.checkpoint_step.unwrap_or_else(|| "2000".to_string());
    let log_file = cfg.logs_dir.join(format!("preload_{}.log", req.task_id));

    let cmd = if root.join("scripts/task.sh").exists() {
        let mut c = Command::new("bash");
        c.arg(root.join("scripts/task.sh"))
            .arg("--checkpoint-step")
            .arg(&step)
            .arg("server")
            .current_dir(&root);
        c
    } else if root.join("scripts/policy_server.sh").exists() {
        let mut c = Command::new("bash");
        c.arg(root.join("scripts/policy_server.sh"))
            .arg("start")
            .arg(&req.task_id)
            .current_dir(&root);
        c
    } else {
        return Json(json!({
            "ok": false,
            "message": format!("未在 {} 下找到 scripts/task.sh 或 scripts/policy_server.sh", root.display()),
        }));
    };

    state
        .append_log(
            "PRELOAD",
            format!(
                "正在预加载模型进 GPU: task={} step={} gpu={} port={} (首次加载约需 60-90 秒，加载完成后常驻显存)",
                req.task_id, step, gpu, port
            ),
        )
        .await;

    match spawn_logged_process(state.clone(), format!("GPU-{}", req.task_id), cmd, log_file.clone(), None).await {
        Ok(pid) => {
            let key = format!("{}:{}", root.display(), req.task_id);
            let info = ManagedProcessInfo {
                name: format!("preload:{}:{}", req.task_id, step),
                pid,
                cmd_summary: format!("Policy Server {} (step {}, GPU {}, port {})", req.task_id, step, gpu, port),
                started_at: now_sec(),
                log_file: log_file.to_string_lossy().to_string(),
                extra: json!({"task_id": req.task_id, "step": step, "port": port, "gpu": gpu}),
            };
            state.preload_procs.lock().await.insert(key, info);
            Json(json!({
                "ok": true,
                "pid": pid,
                "port": port,
                "gpu": gpu,
                "message": format!("已启动 GPU 预加载 (PID {})，请等待状态变为『GPU 已就绪』后点击开始部署。", pid),
            }))
        }
        Err(e) => Json(json!({"ok": false, "message": format!("启动 GPU 预加载失败: {}", e)})),
    }
}

#[derive(Deserialize)]
struct UnloadDeployReq {
    deploy_root: String,
    task_id: String,
}

async fn api_deploy_unload_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<UnloadDeployReq>,
) -> Json<Value> {
    let root = PathBuf::from(req.deploy_root.trim());
    let (port, gpu) = known_task_port_gpu(&req.task_id);

    if root.join("scripts/task.sh").exists() {
        let _ = Command::new("bash")
            .arg(root.join("scripts/task.sh"))
            .arg("stop")
            .current_dir(&root)
            .output()
            .await;
    } else if root.join("scripts/policy_server.sh").exists() {
        let _ = Command::new("bash")
            .arg(root.join("scripts/policy_server.sh"))
            .arg("stop")
            .arg(&req.task_id)
            .current_dir(&root)
            .output()
            .await;
    }

    if let Some(pid) = find_pid_on_port(port) {
        kill_process_group_or_pid(pid);
    }

    let key = format!("{}:{}", root.display(), req.task_id);
    if let Some(p) = state.preload_procs.lock().await.remove(&key) {
        kill_process_group_or_pid(p.pid);
    }

    state
        .append_log(
            "PRELOAD",
            format!("已卸载任务 {} 的 Policy Server (端口 {}, GPU {} 已释放)", req.task_id, port, gpu),
        )
        .await;
    Json(json!({"ok": true, "message": format!("已卸载任务 {} 的 GPU 模型服务", req.task_id)}))
}

#[derive(Deserialize)]
struct StartDeployReq {
    deploy_root: String,
    task_id: String,
    checkpoint_step: String,
    checkpoint_path: Option<String>,
    prompt: Option<String>,
    launch_mode: Option<String>, // "hil_fast" | "original_script"
    enable_hil: Option<bool>,
    auto_resume: Option<bool>,
    idle_resume_sec: Option<f64>,
    spacemouse_deadzone: Option<f64>,
    record_mode: Option<String>, // "none" | "hil_only" | "full_and_hil"
    record_dir: Option<String>,
    skip_reset: Option<bool>,
    cam_width: Option<u32>,
    cam_height: Option<u32>,
    fps: Option<u32>,
}

async fn api_deploy_start_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<StartDeployReq>,
) -> Json<Value> {
    {
        let rol = state.rollout_proc.lock().await;
        if let Some(ref p) = *rol {
            if pid_alive(p.pid) {
                return Json(json!({"ok": false, "message": format!("部署任务已在运行中 (PID {})", p.pid)}));
            }
        }
    }
    {
        let rec = state.record_proc.lock().await;
        if let Some(ref p) = *rec {
            if pid_alive(p.pid) {
                return Json(json!({"ok": false, "message": format!("当前有正在运行的录制进程 (PID {})，请先停止录制。", p.pid)}));
            }
        }
    }

    let cfg = state.config.lock().await.clone();
    let root = PathBuf::from(req.deploy_root.trim());
    if !root.exists() {
        return Json(json!({"ok": false, "message": format!("部署目录不存在: {}", root.display())}));
    }

    let (port, gpu) = known_task_port_gpu(&req.task_id);
    let launch_mode = req.launch_mode.unwrap_or_else(|| "hil_fast".to_string());
    let log_file = cfg.logs_dir.join("rollout.log");
    let control_file = cfg.runtime_dir.join("deploy_control.json");
    let status_file = cfg.runtime_dir.join("deploy_status.json");
    let _ = std::fs::remove_file(&control_file);

    // Clear any residual STOP file in deployment root
    let _ = std::fs::remove_file(root.join("runtime/STOP"));

    let cmd = if launch_mode == "original_script" {
        if root.join("scripts/task.sh").exists() {
            let mut c = Command::new("bash");
            c.arg(root.join("scripts/task.sh"))
                .arg("--checkpoint-step")
                .arg(&req.checkpoint_step)
                .arg("start")
                .current_dir(&root);
            c
        } else if root.join("run_task.sh").exists() {
            let mut c = Command::new("bash");
            c.arg(root.join("run_task.sh"))
                .arg(&req.task_id)
                .current_dir(&root);
            c
        } else {
            return Json(json!({"ok": false, "message": "未找到原版启动脚本 (scripts/task.sh 或 run_task.sh)"}));
        }
    } else {
        // Ensure GPU Policy Server is already warm; if not, trigger preload automatically first!
        if !http_healthz_ok(port).await {
            return Json(json!({
                "ok": false,
                "need_preload": true,
                "port": port,
                "gpu": gpu,
                "message": format!(
                    "任务 {} 的 Policy Server (端口 {}) 尚未加载进 GPU。请先点击『1. 预加载模型进 GPU』，待状态变为就绪后点击『2. 开始部署』即可秒级启动！（或切换为『原版脚本模式』）",
                    req.task_id, port
                ),
            }));
        }

        let py = python_bin_for_env(&cfg.conda_env);
        let helper = cfg.project_root.join("scripts/franka_helper.py");
        let ckpt = req.checkpoint_path.unwrap_or_else(|| {
            root.join("checkpoints")
                .join(&req.task_id)
                .join(&req.checkpoint_step)
                .to_string_lossy()
                .to_string()
        });
        let prompt = req.prompt.unwrap_or_else(|| {
            "Insert the green rectangular block from the table into the matching hole in the blue pegboard.".to_string()
        });
        let rec_dir = req
            .record_dir
            .filter(|s| !s.trim().is_empty())
            .unwrap_or_else(|| cfg.storage_root.join("hil_recordings").to_string_lossy().to_string());

        let mut c = Command::new(&py);
        c.arg("-u")
            .arg(&helper)
            .arg("hil-rollout")
            .arg("--teleop-root")
            .arg(&cfg.teleop_root)
            .arg("--deploy-root")
            .arg(&root)
            .arg("--task")
            .arg(&req.task_id)
            .arg("--prompt")
            .arg(&prompt)
            .arg("--checkpoint")
            .arg(&ckpt)
            .arg("--port")
            .arg(port.to_string())
            .arg("--nuc-ip")
            .arg(&cfg.nuc_ip)
            .arg("--cam-width")
            .arg(req.cam_width.unwrap_or(424).to_string())
            .arg("--cam-height")
            .arg(req.cam_height.unwrap_or(240).to_string())
            .arg("--fps")
            .arg(req.fps.unwrap_or(30).to_string())
            .arg("--enable-hil")
            .arg(if req.enable_hil.unwrap_or(true) { "1" } else { "0" })
            .arg("--auto-resume")
            .arg(if req.auto_resume.unwrap_or(true) { "1" } else { "0" })
            .arg("--idle-resume-sec")
            .arg(req.idle_resume_sec.unwrap_or(1.2).to_string())
            .arg("--spacemouse-deadzone")
            .arg(req.spacemouse_deadzone.unwrap_or(0.15).to_string())
            .arg("--record-mode")
            .arg(req.record_mode.clone().unwrap_or_else(|| "hil_only".to_string()))
            .arg("--record-dir")
            .arg(&rec_dir)
            .arg("--preview-dir")
            .arg(&cfg.preview_dir)
            .arg("--control-file")
            .arg(&control_file)
            .arg("--status-file")
            .arg(&status_file)
            .current_dir(&cfg.teleop_root);

        if req.skip_reset.unwrap_or(false) {
            c.arg("--skip-reset");
        }
        // Also add junfeng/.venv site-packages to PYTHONPATH so openpi_client is available in conda env
        let junfeng_sp = cfg
            .teleop_root
            .join("junfeng/.venv/lib/python3.12/site-packages");
        if junfeng_sp.exists() {
            c.env("PYTHONPATH", junfeng_sp.to_string_lossy().to_string());
        }
        c
    };

    match spawn_logged_process(state.clone(), "DEPLOY".to_string(), cmd, log_file.clone(), None).await {
        Ok(pid) => {
            let info = ManagedProcessInfo {
                name: format!("deploy:{}:{}", req.task_id, req.checkpoint_step),
                pid,
                cmd_summary: format!("Rollout {} (step {}, mode={})", req.task_id, req.checkpoint_step, launch_mode),
                started_at: now_sec(),
                log_file: log_file.to_string_lossy().to_string(),
                extra: json!({
                    "deploy_root": root.to_string_lossy(),
                    "task_id": req.task_id,
                    "checkpoint_step": req.checkpoint_step,
                    "launch_mode": launch_mode,
                    "port": port,
                    "gpu": gpu,
                }),
            };
            *state.rollout_proc.lock().await = Some(info.clone());
            state
                .append_log(
                    "DEPLOY",
                    format!(
                        "已启动推理部署 (PID {}): task={} step={} mode={}",
                        pid, req.task_id, req.checkpoint_step, launch_mode
                    ),
                )
                .await;
            Json(json!({"ok": true, "pid": pid, "info": info, "message": "部署已启动"}))
        }
        Err(e) => Json(json!({"ok": false, "message": format!("启动部署失败: {}", e)})),
    }
}

async fn api_deploy_control_handler(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<Value>,
) -> Json<Value> {
    let cfg = state.config.lock().await.clone();
    let act = payload["action"].as_str().unwrap_or("");

    let control_file = cfg.runtime_dir.join("deploy_control.json");
    let _ = std::fs::write(&control_file, payload.to_string());

    if act == "stop" {
        let mut rol = state.rollout_proc.lock().await;
        if let Some(p) = rol.take() {
            if let Some(dr) = p.extra["deploy_root"].as_str() {
                let stop_marker = PathBuf::from(dr).join("runtime/STOP");
                if let Some(parent) = stop_marker.parent() {
                    let _ = std::fs::create_dir_all(parent);
                }
                let _ = std::fs::write(&stop_marker, "");
            }
            tokio::spawn(async move {
                tokio::time::sleep(Duration::from_millis(600)).await;
                kill_process_group_or_pid(p.pid);
            });
            state
                .append_log("DEPLOY", format!("已停止部署推理进程 (PID {})，GPU 模型保持常驻", p.pid))
                .await;
            return Json(json!({"ok": true, "message": "已停止部署推理（GPU 模型仍保持预热状态，可随时再次开始）"}));
        }
    }

    state
        .append_log("DEPLOY", format!("已发送部署实时控制指令: {}", act))
        .await;
    Json(json!({"ok": true, "message": format!("已发送部署指令: {}", act)}))
}

async fn api_camera_image_handler(
    State(state): State<Arc<AppState>>,
    AxumPath(cam_name): AxumPath<String>,
) -> Response {
    let cfg = state.config.lock().await.clone();
    let cam_info = find_latest_camera_images(&cfg);
    let key = if cam_name.starts_with("wrist") {
        "wrist_path"
    } else {
        "exterior_path"
    };

    if let Some(path_str) = cam_info[key].as_str() {
        if let Ok(bytes) = tokio::fs::read(path_str).await {
            return (
                [
                    (header::CONTENT_TYPE, "image/jpeg"),
                    (header::CACHE_CONTROL, "no-store, no-cache, must-revalidate"),
                ],
                bytes,
            )
                .into_response();
        }
    }

    // Return a clean SVG placeholder if no frame is on disk yet
    let label = if cam_name.starts_with("wrist") {
        "Wrist Camera (腕部相机 CPC93630001C)"
    } else {
        "Exterior Camera (外部相机 CPC936300041)"
    };
    let svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="424" height="240" viewBox="0 0 424 240">
        <rect width="100%" height="100%" fill="#0d1117"/>
        <text x="50%" y="46%" dominant-baseline="middle" text-anchor="middle" fill="#8b949e" font-family="system-ui,sans-serif" font-size="14">{}</text>
        <text x="50%" y="58%" dominant-baseline="middle" text-anchor="middle" fill="#484f58" font-family="system-ui,sans-serif" font-size="12">等待实时观测或点击「抓取相机快照」</text>
        </svg>"##,
        label
    );
    (
        [
            (header::CONTENT_TYPE, "image/svg+xml; charset=utf-8"),
            (header::CACHE_CONTROL, "no-store"),
        ],
        svg,
    )
        .into_response()
}

pub fn build_router(state: Arc<AppState>) -> Router {
    Router::new()
        .route("/", get(index_handler))
        .route("/styles.css", get(css_handler))
        .route("/app.js", get(js_handler))
        .route("/api/state", get(api_state_handler))
        .route("/api/config/update", post(api_update_config_handler))
        .route("/api/device/command", post(api_device_command_handler))
        .route("/api/device/desk", post(api_desk_action_handler))
        .route("/api/record/start", post(api_record_start_handler))
        .route("/api/record/control", post(api_record_control_handler))
        .route("/api/datasets", get(api_datasets_list_handler))
        .route("/api/datasets/preview", post(api_dataset_preview_handler))
        .route("/api/datasets/replay", post(api_dataset_replay_handler))
        .route("/api/deploy/preload", post(api_deploy_preload_handler))
        .route("/api/deploy/unload", post(api_deploy_unload_handler))
        .route("/api/deploy/start", post(api_deploy_start_handler))
        .route("/api/deploy/control", post(api_deploy_control_handler))
        .route("/api/camera/{cam_name}", get(api_camera_image_handler))
        .layer(CorsLayer::permissive())
        .with_state(state)
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let cli = Cli::parse();
    let exe_dir = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let runtime_dir = exe_dir.join("runtime");
    let preview_dir = runtime_dir.join("preview");
    let logs_dir = runtime_dir.join("logs");
    std::fs::create_dir_all(&preview_dir)?;
    std::fs::create_dir_all(&logs_dir)?;

    let default_storage = PathBuf::from("/data2/yjd/.cache/huggingface/lerobot");
    let storage_root = if default_storage.exists() {
        default_storage
    } else {
        runtime_dir.join("datasets")
    };
    let _ = std::fs::create_dir_all(&storage_root);

    let app_cfg = AppConfig {
        project_root: exe_dir.clone(),
        runtime_dir,
        preview_dir,
        logs_dir,
        teleop_root: PathBuf::from(&cli.teleop_root),
        conda_env: PathBuf::from(&cli.conda_env),
        storage_root,
        nuc_ip: "192.168.50.10".to_string(),
        nuc_port: 4242,
        desk_port: 8443,
        custom_deploy_roots: Vec::new(),
    };

    match cli.command {
        Some(Commands::Status { server }) => {
            println!("Querying Franka Web at {} ...", server);
            let status_out = Command::new("curl")
                .args(["-fsS", &format!("{}/api/state", server.trim_end_matches('/'))])
                .output()
                .await?;
            println!("{}", String::from_utf8_lossy(&status_out.stdout));
            return Ok(());
        }
        Some(Commands::Serve { host, port }) => {
            run_server(app_cfg, &host, port).await?;
        }
        None => {
            run_server(app_cfg, &cli.host, cli.port).await?;
        }
    }

    Ok(())
}

async fn run_server(cfg: AppConfig, host: &str, port: u16) -> Result<()> {
    let state = Arc::new(AppState {
        config: Mutex::new(cfg),
        logs: Mutex::new(VecDeque::with_capacity(MAX_LOG_LINES)),
        record_proc: Mutex::new(None),
        rollout_proc: Mutex::new(None),
        preload_procs: Mutex::new(HashMap::new()),
        cached_robot_state: Mutex::new((0.0, json!({}))),
    });

    state
        .append_log(
            "SYSTEM",
            format!("Franka Web 服务已启动，监听 http://{}:{}", host, port),
        )
        .await;

    let app = build_router(state);
    let addr: SocketAddr = format!("{}:{}", host, port).parse()?;
    tracing::info!("Franka Web listening on http://{}", addr);
    let listener = tokio::net::TcpListener::bind(addr).await?;
    axum::serve(listener, app).await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_known_task_port_gpu() {
        assert_eq!(known_task_port_gpu("green_block_insertion"), (8141, "3".to_string()));
        assert_eq!(known_task_port_gpu("in_the_pot"), (8128, "3".to_string()));
        assert_eq!(known_task_port_gpu("book_shelf"), (8127, "2".to_string()));
    }

    #[test]
    fn test_scan_datasets_empty_dir() {
        let tmp = std::env::temp_dir().join("franka_web_test_empty_ds");
        let _ = std::fs::create_dir_all(&tmp);
        let list = scan_datasets_sync(&tmp);
        assert!(list.is_empty());
    }
}
