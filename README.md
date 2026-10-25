# Franka Web

用 Rust (`axum` + `tokio`) 为 Franka Research 3 机器人开发的操作网页与 CLI 工具，参考 [cobot-web](https://github.com/ajwwja777/cobot-web) 的使用体验与模块组织，复用现场 `Franka-Teleop` 与 `OpenPI` 能力。

## 核心功能（首版）

1. **采集/部署前基本控制 (`Device & Pre-Capture Control`)**
   - **Conda / Python 环境选择**：自动扫描并支持切换现场 Conda 环境（如 `/data2/yjd/.conda/envs/franka`）与虚拟环境。
   - **Franka Desk 隧道、关节锁与 FCI 控制**：支持一键调用 `franka-connect` / `franka-disconnect` 打开/关闭 Desk 隧道（`https://localhost:8443/desk/`），并提供 Desk API **Unlock 解锁关节 / Lock 锁定关节 / Activate FCI / Deactivate FCI** 按钮及 Desk 页面直达链接。
   - **NUC 服务与机械臂基础操作**：直接封装调用原有 `franka-start`、`franka-stop`、`franka-status`、`franka-reset`（归位并打开夹爪）、`franka-gripper-reinit` 及 `franka-record --check`（五项预检）。
   - **实时机械臂状态与双相机观测**：只读查询 `192.168.50.10:4242` zerorpc（7 轴关节角、6D 末端位姿、夹爪开度，绝不触发有副作用的控制器启动），并实时展示外部相机（`CPC936300041`）与腕部相机（`CPC93630001C`）画面及 4×RTX 4090 显存状态。

2. **遥操作录制与已有录制加载 (`Recording & Dataset Browser`)**
   - **点击即可录制**：支持在网页自定义存储根目录（`HF_LEROBOT_HOME`）、`repo_id`、任务描述 Prompt、计划条数、FPS 及续录（`resume`）配置。
   - **网页录制控制与实时预览**：录制期间实时将双相机画面推送到网页预览框，并支持在网页直接点击 **完成本条/下一步 (`→`)**、**丢弃重录本条 (`←`)**、**结束录制并保存 (`Esc`)**（与物理键盘方向键完全等效）。
   - **加载已有录制到网页**：自动扫描存储路径下所有 LeRobot 数据集与 HIL 录制轨迹，支持一键提取任意 Episode 的双相机画面与状态/动作数据加载到网页中预览，或直接选为续录目标、触发 `franka-replay` 真机回放。

3. **模型部署、GPU 提前预加载与 SpaceMouse HIL 打断录制 (`Deployment, GPU Preload & HIL`)**
   - **可选部署脚本目录与 Checkpoint 自动解析**：支持选择或添加任意部署目录（如 `xiqing/green_block_insertion`、`junfeng` 等），自动展示其任务、Checkpoint 路径、Step、分配的 GPU 与端口。
   - **两阶段极速部署（提前加载进 GPU，点击开始立即部署）**：
     - **1. 预加载模型进 GPU (Preload)**：后台拉起对应任务的 Policy Server，将权重提前加载进对应 RTX 4090 并常驻监听（避免每次部署重复等待 80 秒冷启动）。
     - **2. 点击开始部署推理 (Start)**：直接复用已就绪的本地 GPU 服务，秒级启动推理。
   - **实时暂停/继续、SpaceMouse HIL 自动打断与部署录制**：
     - 支持网页按钮或快捷键（`Alt+P`）随时**直接暂停/继续推理**。
     - **SpaceMouse HIL 打断**：在部署推理时，若推动 SpaceMouse（或按夹爪键），自动暂停模型动作输出、无缝切换为 SpaceMouse 遥操作并自动录制 HIL 部分；支持勾选 **“SpaceMouse 停止后自动恢复推理”**（可设静止恢复延迟秒数）或保持暂停由按键/按钮手动恢复。
     - **部署时录制**：可选 `仅自动录制 HIL 接管片段 (hil_only)`、`录制完整推理 + HIL 全过程 (full_and_hil)` 或 `不录制 (none)`，存储路径可自由选择。

## 快速使用

```bash
# 启动网页服务（默认监听 0.0.0.0:8016）
./scripts/ui_up.sh

# 查看服务运行状态
./scripts/ui_status.sh

# 停止网页服务
./scripts/ui_down.sh
```

在 Franka 本机浏览器打开 `http://127.0.0.1:8016`（或通过 SSH 端口转发 `ssh -L 8016:127.0.0.1:8016 yjd@10.4.0.136` 在本地浏览器打开 `http://127.0.0.1:8016`）。

## 目录结构

```text
franka-web/
├── Cargo.toml
├── src/main.rs             # Rust 服务端 (Axum) 与 CLI 入口
├── frontend/               # 原生模块化前端 (index.html, styles.css, app.js)
├── scripts/
│   ├── franka_helper.py    # Python 桥接脚本 (只读状态/Desk API/数据集加载/录制包装/HIL部署)
│   ├── ui_up.sh            # 启动网页服务
│   ├── ui_down.sh          # 停止网页服务
│   └── ui_status.sh        # 查看网页服务状态
└── runtime/                # 运行时日志、PID 与相机预览缓存（不入 Git）
```
