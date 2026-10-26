#!/usr/bin/env python3
"""
Franka Web Bridge Helper
Provides:
1. robot-state: Read-only probe of NUC zerorpc (joint positions, ee pose, gripper), USB devices, Desk tunnel.
2. desk-action: Franka Desk REST API helper for lock/unlock brakes and FCI activation.
3. camera-snapshot: Grab a single read-only dual-camera snapshot.
4. camera-stream: Continuous read-only dual-camera live stream with real-time Hz & latency measurement.
5. dataset-preview: Load an existing LeRobot dataset episode for web visualization (frames + state/action summary).
6. record-wrapper: Run Franka-Teleop recording with custom storage path, live web camera preview + real-time capture Hz, and web button controls.
7. hil-rollout: Fast deployment runner reusing preloaded GPU Policy Server, with Pause/Resume, SpaceMouse HIL interrupt, and HIL/rollout recording.
"""

from __future__ import annotations

import argparse
import base64
import collections
import datetime as dt
import json
import math
import os
import pathlib
import signal
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from typing import Any


def _port_open(host: str, port: int, timeout: float = 1.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def _save_jpeg(path: pathlib.Path, rgb_array: Any, quality: int = 80) -> None:
    try:
        from PIL import Image
        import numpy as np

        arr = np.asarray(rgb_array)
        if arr.dtype != np.uint8:
            arr = np.clip(arr, 0, 255).astype(np.uint8)
        tmp = path.with_suffix(".tmp.jpg")
        Image.fromarray(arr, mode="RGB").save(tmp, format="JPEG", quality=quality)
        tmp.replace(path)
    except Exception:
        pass


def _write_preview_meta(preview_dir: pathlib.Path, meta: dict[str, Any]) -> None:
    try:
        tmp = preview_dir / "meta.tmp.json"
        tmp.write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
        tmp.replace(preview_dir / "meta.json")
    except Exception:
        pass


def cmd_robot_state(args: argparse.Namespace) -> int:
    nuc_ip = args.nuc_ip
    nuc_port = args.nuc_port
    desk_port = args.desk_port

    t0 = time.perf_counter()
    nuc_online = _port_open(nuc_ip, nuc_port, timeout=1.2)
    desk_tunnel_online = _port_open("127.0.0.1", desk_port, timeout=0.6)

    usb_out = ""
    try:
        usb_out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=2).stdout.lower()
    except Exception:
        pass

    spacemouse_connected = "3dconnexion" in usb_out
    orbbec_count = usb_out.count("2bc5:0807")

    result: dict[str, Any] = {
        "nuc_ip": nuc_ip,
        "nuc_port": nuc_port,
        "nuc_online": nuc_online,
        "desk_tunnel_online": desk_tunnel_online,
        "desk_url": f"https://localhost:{desk_port}/desk/",
        "spacemouse_connected": spacemouse_connected,
        "orbbec_count": orbbec_count,
        "robot_readable": False,
        "fci_active": False,
        "joint_positions": None,
        "joint_positions_deg": None,
        "ee_pose": None,
        "gripper_width_m": None,
        "gripper_norm": None,
        "probe_ms": round((time.perf_counter() - t0) * 1000.0, 1),
        "error": None,
    }

    if not nuc_online:
        result["error"] = f"NUC {nuc_ip}:{nuc_port} 未连通（请先执行 franka-start）"
        print(json.dumps(result, ensure_ascii=False))
        return 0

    try:
        teleop_root = pathlib.Path(args.teleop_root)
        if (teleop_root / "interface").exists():
            sys.path.insert(0, str(teleop_root / "interface"))
        import zerorpc
        import numpy as np

        client = zerorpc.Client(timeout=3, heartbeat=5)
        client.connect(f"tcp://{nuc_ip}:{nuc_port}")
        try:
            jp = client.robot_get_joint_positions()
            ee = client.robot_get_ee_pose()
            if jp is not None and len(jp) == 7 and np.isfinite(jp).all():
                jp_list = [round(float(x), 5) for x in jp]
                result["joint_positions"] = jp_list
                result["joint_positions_deg"] = [round(float(x) * 180.0 / math.pi, 2) for x in jp]
                result["robot_readable"] = True
                result["fci_active"] = True
            if ee is not None and len(ee) == 6 and np.isfinite(ee).all():
                result["ee_pose"] = [round(float(x), 5) for x in ee]
            try:
                gstate = client.gripper_get_state()
                if isinstance(gstate, dict) and "width" in gstate:
                    w = float(gstate["width"])
                    result["gripper_width_m"] = round(w, 5)
                    result["gripper_norm"] = round(min(max(w / 0.085, 0.0), 1.0), 4)
            except Exception:
                pass
        finally:
            try:
                client.close()
            except Exception:
                pass
    except Exception as exc:
        result["error"] = f"读取机械臂状态失败（请检查 Desk 解锁与 FCI 是否开启）: {exc}"

    result["probe_ms"] = round((time.perf_counter() - t0) * 1000.0, 1)
    print(json.dumps(result, ensure_ascii=False))
    return 0


class _DeskClient:
    """Minimal Franka Desk REST API client."""

    def __init__(self, base_url: str, username: str = "", password: str = ""):
        self.base_url = base_url.rstrip("/")
        self.username = username
        self.password = password
        self.ctx = ssl.create_default_context()
        self.ctx.check_hostname = False
        self.ctx.verify_mode = ssl.CERT_NONE
        self.token: str | None = None

    def _request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
        headers: dict[str, str] | None = None,
        timeout: float = 15.0,
    ) -> tuple[int, str]:
        url = f"{self.base_url}{path}"
        data = None
        req_headers = {"Accept": "application/json"}
        if self.token:
            req_headers["Cookie"] = f"authorization={self.token}"
            req_headers["X-Control-Token"] = self.token
        if headers:
            req_headers.update(headers)
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            req_headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, headers=req_headers, method=method)
        try:
            with urllib.request.urlopen(req, context=self.ctx, timeout=timeout) as resp:
                return resp.status, resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as e:
            body = ""
            try:
                body = e.read().decode("utf-8", errors="replace")
            except Exception:
                pass
            return e.code, body

    def login_if_needed(self) -> None:
        if not self.username or not self.password:
            return
        import hashlib

        raw = f"{self.password}#{self.username}@franka".encode("utf-8")
        digest = hashlib.sha256(raw).digest()
        encoded_pw = base64.b64encode(digest).decode("ascii")
        code, body = self._request(
            "POST",
            "/admin/api/login",
            {"login": self.username, "password": encoded_pw},
        )
        if code == 200 and body:
            self.token = body.strip().strip('"')


def cmd_desk_action(args: argparse.Namespace) -> int:
    action = args.action
    desk_port = args.desk_port
    base_url = f"https://127.0.0.1:{desk_port}"

    if not _port_open("127.0.0.1", desk_port, timeout=1.0):
        print(
            json.dumps(
                {
                    "ok": False,
                    "action": action,
                    "message": f"Franka Desk 隧道 (localhost:{desk_port}) 未开启，请先点击『Connect Desk 隧道』。",
                    "desk_url": f"https://localhost:{desk_port}/desk/",
                },
                ensure_ascii=False,
            )
        )
        return 1

    auth_file = pathlib.Path(args.runtime_dir) / "desk_auth.json"
    username = args.username or ""
    password = args.password or ""
    if auth_file.is_file() and (not username or not password):
        try:
            saved = json.loads(auth_file.read_text(encoding="utf-8"))
            username = username or saved.get("username", "")
            password = password or saved.get("password", "")
        except Exception:
            pass

    if args.save_credentials and username and password:
        auth_file.parent.mkdir(parents=True, exist_ok=True)
        auth_file.write_text(json.dumps({"username": username, "password": password}), encoding="utf-8")
        os.chmod(auth_file, 0o600)

    client = _DeskClient(base_url, username=username, password=password)
    try:
        client.login_if_needed()
        if action == "unlock":
            code, body = client._request("POST", "/desk/api/robot/open-brakes", {"force": True}, timeout=35.0)
        elif action == "lock":
            code, body = client._request("POST", "/desk/api/robot/close-brakes", {"force": True}, timeout=25.0)
        elif action == "fci-on":
            code, body = client._request("POST", "/admin/api/fci", {"token": client.token or ""}, timeout=10.0)
            if code not in (200, 204):
                code, body = client._request("POST", "/desk/api/navigation/fci", {"active": True}, timeout=10.0)
        elif action == "fci-off":
            code, body = client._request("DELETE", "/admin/api/fci", {"token": client.token or ""}, timeout=10.0)
        else:
            code, body = client._request("GET", "/admin/api/control-token/status", timeout=5.0)

        ok = 200 <= code < 300
        msg = (
            f"Desk 操作 `{action}` 成功 (HTTP {code})"
            if ok
            else f"Desk API 返回 HTTP {code}（若需首次确认，请在 https://localhost:{desk_port}/desk/ 操作或配置 Desk 账号密码）: {body[:160]}"
        )
        print(
            json.dumps(
                {
                    "ok": ok,
                    "action": action,
                    "http_code": code,
                    "message": msg,
                    "desk_url": f"https://localhost:{desk_port}/desk/",
                },
                ensure_ascii=False,
            )
        )
        return 0 if ok else 1
    except Exception as exc:
        print(
            json.dumps(
                {
                    "ok": False,
                    "action": action,
                    "message": f"调用 Franka Desk 接口异常: {exc}。可直接在 https://localhost:{desk_port}/desk/ 操作。",
                    "desk_url": f"https://localhost:{desk_port}/desk/",
                },
                ensure_ascii=False,
            )
        )
        return 1


def cmd_camera_snapshot(args: argparse.Namespace) -> int:
    """Capture one snapshot from exterior & wrist Orbbec cameras into preview_dir."""
    preview_dir = pathlib.Path(args.preview_dir)
    preview_dir.mkdir(parents=True, exist_ok=True)
    teleop_root = pathlib.Path(args.teleop_root)
    sys.path.insert(0, str(teleop_root))

    try:
        import yaml
        from scripts.core.record import RecordConfig, create_camera_configs
        from lerobot.cameras import make_cameras_from_configs

        cfg_path = teleop_root / "scripts" / "config" / "record_config.yaml"
        raw_cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        rcfg = RecordConfig(raw_cfg["record"])
        cam_cfgs = create_camera_configs(rcfg)
        cameras = make_cameras_from_configs(cam_cfgs)
        connected = []
        ext_ms = 0.0
        wrist_ms = 0.0
        try:
            for name, cam in cameras.items():
                cam.connect()
                connected.append(cam)
            time.sleep(0.15)
            for key, out_name in (("exterior_image", "exterior.jpg"), ("wrist_image", "wrist.jpg")):
                if key in cameras:
                    t0 = time.perf_counter()
                    frame = cameras[key].async_read(timeout_ms=500)
                    dt_ms = (time.perf_counter() - t0) * 1000.0
                    if key == "exterior_image":
                        ext_ms = round(dt_ms, 1)
                    else:
                        wrist_ms = round(dt_ms, 1)
                    _save_jpeg(preview_dir / out_name, frame)
        finally:
            for cam in reversed(connected):
                try:
                    cam.disconnect()
                except Exception:
                    pass
        meta = {
            "updated_at": time.time(),
            "frame_seq": int(time.time() * 10) % 100000,
            "source": "camera_snapshot",
            "mode": "snapshot",
            "capture_hz": float(rcfg.fps),
            "target_fps": int(rcfg.fps),
            "width": int(rcfg.width),
            "height": int(rcfg.height),
            "exterior_ms": ext_ms,
            "wrist_ms": wrist_ms,
            "message": "已抓取实时双相机快照",
        }
        _write_preview_meta(preview_dir, meta)
        print(json.dumps({"ok": True, **meta}, ensure_ascii=False))
        return 0
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1


def cmd_camera_stream(args: argparse.Namespace) -> int:
    """
    Continuous read-only camera preview stream when no recording/rollout is active.
    Reads at hardware FPS (e.g. 30Hz) to measure real camera capture Hz, and writes JPEG preview frames at ~8Hz.
    Automatically exits on SIGTERM/SIGINT so recording/deployment can take over cameras immediately.
    """
    preview_dir = pathlib.Path(args.preview_dir)
    preview_dir.mkdir(parents=True, exist_ok=True)
    teleop_root = pathlib.Path(args.teleop_root)
    sys.path.insert(0, str(teleop_root))

    stop_flag = False

    def _handle_sig(_signum: int, _frame: Any) -> None:
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGTERM, _handle_sig)
    signal.signal(signal.SIGINT, _handle_sig)

    try:
        try:
            import pyorbbecsdk as ob

            ob.Context().set_logger_level(ob.OBLogLevel.ERROR)
        except Exception:
            pass

        import yaml
        from scripts.core.record import RecordConfig, create_camera_configs
        from lerobot.cameras import make_cameras_from_configs

        cfg_path = teleop_root / "scripts" / "config" / "record_config.yaml"
        raw_cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        rcfg = RecordConfig(raw_cfg["record"])
        target_fps = int(rcfg.fps) if int(rcfg.fps) > 0 else 30
        cam_cfgs = create_camera_configs(rcfg)
        cameras = make_cameras_from_configs(cam_cfgs)

        connected = []
        try:
            for name, cam in cameras.items():
                cam.connect()
                connected.append(cam)
            print(
                f"[CAM-STREAM] 双相机实时流已启动 ({rcfg.width}×{rcfg.height} @ 目标 {target_fps}Hz)",
                flush=True,
            )

            obs_times: collections.deque[float] = collections.deque(maxlen=45)
            frame_seq = 0
            last_jpeg_write = 0.0
            jpeg_interval = 1.0 / max(1.0, float(args.preview_fps))

            stop_file = pathlib.Path(args.stop_file) if getattr(args, "stop_file", "") else None
            while not stop_flag:
                if stop_file is not None and stop_file.exists():
                    break
                loop_t0 = time.perf_counter()
                ext_frame = None
                wrist_frame = None
                ext_ms = 0.0
                wrist_ms = 0.0

                if "exterior_image" in cameras:
                    t_c = time.perf_counter()
                    ext_frame = cameras["exterior_image"].async_read(timeout_ms=250)
                    ext_ms = round((time.perf_counter() - t_c) * 1000.0, 1)
                if "wrist_image" in cameras:
                    t_c = time.perf_counter()
                    wrist_frame = cameras["wrist_image"].async_read(timeout_ms=250)
                    wrist_ms = round((time.perf_counter() - t_c) * 1000.0, 1)

                now_mono = time.perf_counter()
                obs_times.append(now_mono)
                actual_hz = 0.0
                if len(obs_times) >= 2 and (obs_times[-1] - obs_times[0]) > 1e-6:
                    actual_hz = round((len(obs_times) - 1) / (obs_times[-1] - obs_times[0]), 1)

                if now_mono - last_jpeg_write >= jpeg_interval:
                    last_jpeg_write = now_mono
                    frame_seq += 1
                    if ext_frame is not None:
                        _save_jpeg(preview_dir / "exterior.jpg", ext_frame)
                    if wrist_frame is not None:
                        _save_jpeg(preview_dir / "wrist.jpg", wrist_frame)
                    _write_preview_meta(
                        preview_dir,
                        {
                            "updated_at": time.time(),
                            "frame_seq": frame_seq,
                            "source": "live_stream",
                            "mode": "live_stream",
                            "capture_hz": actual_hz,
                            "target_fps": target_fps,
                            "width": int(rcfg.width),
                            "height": int(rcfg.height),
                            "exterior_ms": ext_ms,
                            "wrist_ms": wrist_ms,
                            "exterior_id": str(rcfg.exterior_cam_id or "CPC936300041"),
                            "wrist_id": str(rcfg.wrist_cam_id or "CPC93630001C"),
                            "message": f"实时相机流 · 采集频率 {actual_hz:.1f} Hz",
                        },
                    )

                elapsed = time.perf_counter() - loop_t0
                sleep_s = max(0.0, (1.0 / target_fps) - elapsed)
                if sleep_s > 0:
                    time.sleep(sleep_s)
        finally:
            for cam in reversed(connected):
                try:
                    cam.disconnect()
                except Exception:
                    pass
            print("[CAM-STREAM] 双相机实时流已安全释放相机句柄", flush=True)
        return 0
    except Exception as exc:
        print(f"[CAM-STREAM] 相机流异常退出: {exc}", file=sys.stderr, flush=True)
        return 1


def cmd_dataset_episode_preview(args: argparse.Namespace) -> int:
    """Load an episode from an existing LeRobot dataset on disk for web inspection."""
    ds_dir = pathlib.Path(args.dataset_dir).resolve()
    ep_idx = int(args.episode_index)
    preview_dir = pathlib.Path(args.preview_dir)
    preview_dir.mkdir(parents=True, exist_ok=True)

    if not ds_dir.is_dir():
        print(json.dumps({"ok": False, "error": f"数据集目录不存在: {ds_dir}"}, ensure_ascii=False))
        return 1

    info_path = ds_dir / "meta" / "info.json"
    info = {}
    if info_path.is_file():
        try:
            info = json.loads(info_path.read_text(encoding="utf-8"))
        except Exception:
            pass

    parquet_files = sorted(ds_dir.glob("data/**/*.parquet"))
    ext_videos = sorted(ds_dir.glob("videos/**/*exterior*/*.mp4"))
    wrist_videos = sorted(ds_dir.glob("videos/**/*wrist*/*.mp4"))

    summary: dict[str, Any] = {
        "ok": True,
        "dataset_dir": str(ds_dir),
        "dataset_name": ds_dir.name,
        "episode_index": ep_idx,
        "info": info,
        "frame_count": 0,
        "sample_states": [],
        "sample_actions": [],
        "extracted_frames": False,
    }

    from_ts = None
    to_ts = None
    try:
        import pyarrow.parquet as pq

        for pf in parquet_files:
            table = pq.read_table(pf)
            cols = table.column_names
            if "episode_index" in cols:
                ep_col = table["episode_index"].to_pylist()
                indices = [i for i, v in enumerate(ep_col) if int(v) == ep_idx]
                if indices:
                    summary["frame_count"] = len(indices)
                    ts_col = table["timestamp"].to_pylist() if "timestamp" in cols else []
                    if ts_col:
                        from_ts = float(ts_col[indices[0]])
                        to_ts = float(ts_col[indices[-1]])
                    step_stride = max(1, len(indices) // 20)
                    if "observation.state" in cols:
                        st_col = table["observation.state"].to_pylist()
                        summary["sample_states"] = [st_col[i] for i in indices[::step_stride][:20]]
                    if "action" in cols:
                        act_col = table["action"].to_pylist()
                        summary["sample_actions"] = [act_col[i] for i in indices[::step_stride][:20]]
                    break
    except Exception as exc:
        summary["parquet_note"] = str(exc)

    try:
        import av

        for vid_list, out_name in ((ext_videos, "exterior.jpg"), (wrist_videos, "wrist.jpg")):
            if not vid_list:
                continue
            with av.open(str(vid_list[0])) as container:
                stream = container.streams.video[0]
                target_ts = ((from_ts + to_ts) * 0.5) if (from_ts is not None and to_ts is not None) else 0.5
                for frame in container.decode(stream):
                    pts_s = float(frame.pts * stream.time_base) if frame.pts is not None else 0.0
                    if pts_s >= target_ts:
                        arr = frame.to_ndarray(format="rgb24")
                        _save_jpeg(preview_dir / out_name, arr)
                        summary["extracted_frames"] = True
                        break
    except Exception as exc:
        summary["video_note"] = str(exc)

    if summary["extracted_frames"]:
        fps_val = float(info.get("fps", 30))
        _write_preview_meta(
            preview_dir,
            {
                "updated_at": time.time(),
                "frame_seq": int(time.time() * 10) % 100000,
                "source": f"dataset:{ds_dir.name}#ep{ep_idx}",
                "mode": "dataset_preview",
                "capture_hz": fps_val,
                "target_fps": int(fps_val),
                "width": 424,
                "height": 240,
                "exterior_ms": 0.0,
                "wrist_ms": 0.0,
                "message": f"数据集预览: {ds_dir.name} #ep{ep_idx} ({summary['frame_count']} 帧)",
            },
        )

    print(json.dumps(summary, ensure_ascii=False))
    return 0


def cmd_record_wrapper(args: argparse.Namespace) -> int:
    """
    Wrap Franka-Teleop's record.py so that:
    1. Storage root (HF_LEROBOT_HOME) and YAML config are customized from web UI.
    2. Live frames and real-time observation Hz during recording are saved to runtime/preview/.
    3. Web control commands (save_episode, rerecord_episode, stop_recording) via runtime/record_cmd.json
       trigger the exact same events dictionary as physical keyboard arrows!
    """
    teleop_root = pathlib.Path(args.teleop_root).resolve()
    preview_dir = pathlib.Path(args.preview_dir).resolve()
    control_file = pathlib.Path(args.control_file).resolve()
    preview_dir.mkdir(parents=True, exist_ok=True)

    if args.storage_root:
        os.environ["HF_LEROBOT_HOME"] = str(pathlib.Path(args.storage_root).expanduser().resolve())
    os.environ["FRANKA_RECORD_CFG"] = str(pathlib.Path(args.config_file).resolve())
    if "DISPLAY" not in os.environ:
        os.environ["DISPLAY"] = ":2"
    if "XAUTHORITY" not in os.environ and pathlib.Path("/run/user/1007/gdm/Xauthority").exists():
        os.environ["XAUTHORITY"] = "/run/user/1007/gdm/Xauthority"

    sys.path.insert(0, str(teleop_root))
    import yaml
    import scripts.core.record as rec_mod
    import scripts.utils.record_preview as rp_mod

    orig_init_kb = rec_mod.init_record_keyboard_listener

    def patched_init_kb(*, enter_finishes: bool = False):
        try:
            listener, events = orig_init_kb(enter_finishes=False)
        except Exception as exc:
            print(f"[WEB-RECORD] Physical keyboard listener note ({exc}); web buttons remain active.", flush=True)
            listener = None
            events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}

        def poll_web_commands():
            while True:
                time.sleep(0.08)
                if not control_file.is_file():
                    continue
                try:
                    cmd_data = json.loads(control_file.read_text(encoding="utf-8"))
                    control_file.unlink(missing_ok=True)
                    act = cmd_data.get("action", "")
                    if act in ("next", "save", "right"):
                        print("[WEB-RECORD] 收到网页指令: 完成/下一步 (→)", flush=True)
                        events["exit_early"] = True
                    elif act in ("rerecord", "discard", "left"):
                        print("[WEB-RECORD] 收到网页指令: 丢弃并重录当前条 (←)", flush=True)
                        events["rerecord_episode"] = True
                        events["exit_early"] = True
                    elif act in ("stop", "esc"):
                        print("[WEB-RECORD] 收到网页指令: 结束会话并保存退出 (Esc)", flush=True)
                        events["stop_recording"] = True
                        events["exit_early"] = True
                except Exception:
                    pass

        t = threading.Thread(target=poll_web_commands, daemon=True)
        t.start()
        return listener, events

    rec_mod.init_record_keyboard_listener = patched_init_kb

    orig_run_record_loop = rp_mod.run_record_loop
    obs_window: collections.deque[float] = collections.deque(maxlen=45)
    frame_seq_box = [0]

    def patched_run_record_loop(backend, *, preview_fps=10, clock=None, **kwargs):
        robot = kwargs.get("robot")
        target_fps = int(kwargs.get("fps", 30))
        last_web_img = [0.0]
        if robot is not None and hasattr(robot, "get_observation"):
            orig_get_obs = robot.get_observation

            def obs_with_web_preview(*a, **kw):
                t_start = time.perf_counter()
                obs = orig_get_obs(*a, **kw)
                t_end = time.perf_counter()
                obs_ms = round((t_end - t_start) * 1000.0, 1)
                obs_window.append(t_end)
                actual_hz = 0.0
                if len(obs_window) >= 2 and (obs_window[-1] - obs_window[0]) > 1e-6:
                    actual_hz = round((len(obs_window) - 1) / (obs_window[-1] - obs_window[0]), 1)

                if t_end - last_web_img[0] >= 0.12 and isinstance(obs, dict):
                    last_web_img[0] = t_end
                    frame_seq_box[0] += 1
                    w, h = 424, 240
                    if "exterior_image" in obs:
                        ext = obs["exterior_image"]
                        if hasattr(ext, "shape") and len(ext.shape) >= 2:
                            h, w = int(ext.shape[0]), int(ext.shape[1])
                        _save_jpeg(preview_dir / "exterior.jpg", ext)
                    if "wrist_image" in obs:
                        _save_jpeg(preview_dir / "wrist.jpg", obs["wrist_image"])
                    _write_preview_meta(
                        preview_dir,
                        {
                            "updated_at": time.time(),
                            "frame_seq": frame_seq_box[0],
                            "source": "franka-record",
                            "mode": "recording",
                            "capture_hz": actual_hz,
                            "target_fps": target_fps,
                            "width": w,
                            "height": h,
                            "exterior_ms": obs_ms,
                            "wrist_ms": obs_ms,
                            "message": f"正在录制 · 实时采集频率 {actual_hz:.1f} Hz (目标 {target_fps} Hz)",
                        },
                    )
                return obs

            robot.get_observation = obs_with_web_preview
        return orig_run_record_loop(backend, preview_fps=preview_fps, clock=clock, **kwargs)

    rp_mod.run_record_loop = patched_run_record_loop
    rec_mod.run_record_loop = patched_run_record_loop

    with open(args.config_file, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if args.episodes and args.episodes > 0:
        cfg["record"]["task"]["num_episodes"] = int(args.episodes)

    if not rec_mod.run_preflight(cfg["record"]["robot"]["ip"]):
        return 1

    record_cfg = rec_mod.RecordConfig(cfg["record"])
    rec_mod.run_record(record_cfg, enter_finishes=False)
    return 0


def cmd_hil_rollout(args: argparse.Namespace) -> int:
    """
    Enhanced Policy Deployment Rollout Runner:
    - Connects to an already-warm Policy Server on 127.0.0.1:<port> (instant startup!)
    - Real-time camera preview + live capture/control Hz display
    - Live Pause / Resume via web button or hotkey (`runtime/deploy_control.json`)
    - SpaceMouse HIL auto-interruption & optional auto-resume
    - Recording deployment trajectories (`none` | `hil_only` | `full_and_hil`)
    """
    deploy_root = pathlib.Path(args.deploy_root).resolve()
    teleop_root = pathlib.Path(args.teleop_root).resolve()
    preview_dir = pathlib.Path(args.preview_dir).resolve()
    control_file = pathlib.Path(args.control_file).resolve()
    status_file = pathlib.Path(args.status_file).resolve()
    preview_dir.mkdir(parents=True, exist_ok=True)
    status_file.parent.mkdir(parents=True, exist_ok=True)

    sys.path.insert(0, str(teleop_root))
    sys.path.insert(0, str(teleop_root / "interface"))
    sys.path.insert(0, str(teleop_root / "teleoperation"))
    if (deploy_root / "integrations" / "franka" / "runtime").is_dir():
        sys.path.insert(0, str(deploy_root / "integrations" / "franka" / "runtime"))
    sys.path.insert(0, str(deploy_root))

    state_doc: dict[str, Any] = {
        "phase": "initializing",
        "mode": "policy",  # "policy" | "paused" | "hil"
        "step": 0,
        "hil_steps": 0,
        "policy_steps": 0,
        "capture_hz": 0.0,
        "hil_enabled": bool(args.enable_hil),
        "auto_resume": bool(args.auto_resume),
        "idle_resume_sec": float(args.idle_resume_sec),
        "record_mode": args.record_mode,
        "record_dir": args.record_dir,
        "started_at": time.time(),
        "message": "正在连接已预热的 GPU 推理服务与机器人...",
    }

    def publish_state(**updates: Any) -> None:
        state_doc.update(updates)
        state_doc["updated_at"] = time.time()
        try:
            tmp = status_file.with_suffix(".tmp.json")
            tmp.write_text(json.dumps(state_doc, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(status_file)
        except Exception:
            pass

    publish_state()

    import numpy as np
    from interface import FrankaConfig, Franka
    from openpi_client import websocket_client_policy
    from lerobot.cameras.configs import ColorMode, Cv2Rotation
    from lerobot.cameras.orbbec.configuration_orbbec import OrbbecCameraConfig

    print(f"[DEPLOY] 连接本地 GPU Policy Server: 127.0.0.1:{args.port} ...", flush=True)
    policy = websocket_client_policy.WebsocketClientPolicy("127.0.0.1", int(args.port))
    server_meta = policy.get_server_metadata()
    print(f"[DEPLOY] Policy Server 就绪: {json.dumps(str(server_meta))[:200]}", flush=True)

    spacemouse = None
    if args.enable_hil:
        try:
            from teleoperation.spacemouse.spacemouse_robot import SpaceMouseRobot

            spacemouse = SpaceMouseRobot(
                use_gripper=True,
                pose_scaler=[0.02, 0.08],
                channel_signs=[-1, -1, 1, -1, -1, 1],
            )
            print("[DEPLOY] SpaceMouse HIL 接管监听已开启（推动推杆即可随时打断并接管推理）", flush=True)
        except Exception as exc:
            print(f"[DEPLOY] SpaceMouse 初始化提示（将继续无 SpaceMouse 部署）: {exc}", flush=True)
            spacemouse = None

    cam_width = int(args.cam_width)
    cam_height = int(args.cam_height)
    fps = int(args.fps)

    camera_config = {
        "exterior_image": OrbbecCameraConfig(
            serial_number_or_name=args.exterior_cam_id,
            fps=fps,
            width=cam_width,
            height=cam_height,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
        ),
        "wrist_image": OrbbecCameraConfig(
            serial_number_or_name=args.wrist_cam_id,
            fps=fps,
            width=cam_width,
            height=cam_height,
            color_mode=ColorMode.RGB,
            rotation=Cv2Rotation.NO_ROTATION,
        ),
    }

    robot_cfg = FrankaConfig(
        robot_ip=args.nuc_ip,
        cameras=camera_config,
        debug=False,
        close_threshold=0.7,
        use_gripper=True,
        gripper_reverse=False,
        gripper_bin_threshold=0.98,
        gripper_max_open=0.085,
    )
    robot = Franka(robot_cfg)

    rec_root: pathlib.Path | None = None
    rec_frames_ext: pathlib.Path | None = None
    rec_frames_wrist: pathlib.Path | None = None
    rec_steps_file = None
    if args.record_mode in ("hil_only", "full_and_hil") and args.record_dir:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        rec_root = pathlib.Path(args.record_dir).expanduser().resolve() / f"{args.task}_{args.record_mode}_{stamp}"
        rec_frames_ext = rec_root / "images" / "exterior"
        rec_frames_wrist = rec_root / "images" / "wrist"
        rec_frames_ext.mkdir(parents=True, exist_ok=True)
        rec_frames_wrist.mkdir(parents=True, exist_ok=True)
        rec_steps_file = (rec_root / "trajectory.jsonl").open("a", encoding="utf-8")
        (rec_root / "meta.json").write_text(
            json.dumps(
                {
                    "task": args.task,
                    "prompt": args.prompt,
                    "checkpoint": args.checkpoint,
                    "record_mode": args.record_mode,
                    "fps": fps,
                    "created_at": stamp,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        publish_state(active_record_path=str(rec_root))
        print(f"[DEPLOY] 部署/HIL 录制已启用，保存路径: {rec_root}", flush=True)

    stop_flag = False

    def _sig_handler(_signum: int, _frame: Any) -> None:
        nonlocal stop_flag
        stop_flag = True

    signal.signal(signal.SIGTERM, _sig_handler)
    signal.signal(signal.SIGINT, _sig_handler)

    try:
        publish_state(phase="connecting_robot", message="正在连接机械臂与双相机并回 Home 位...")
        robot.connect()
        if not args.skip_reset:
            print("[DEPLOY] 机械臂正在复位到 Home 位...", flush=True)
            robot.reset()

        publish_state(phase="running", mode="policy", message="模型正在实时推理控制中")
        chunk: np.ndarray | None = None
        chunk_idx = 0
        open_loop_horizon = int(args.open_loop_horizon)
        last_spacemouse_active_t = 0.0
        manual_paused = False
        last_preview_t = 0.0
        obs_times: collections.deque[float] = collections.deque(maxlen=45)
        frame_seq = 0

        while not stop_flag:
            t0 = time.perf_counter()

            if control_file.is_file():
                try:
                    cmd = json.loads(control_file.read_text(encoding="utf-8"))
                    control_file.unlink(missing_ok=True)
                    act = cmd.get("action", "")
                    if act == "stop":
                        print("[DEPLOY] 收到网页停止指令，正在安全退出...", flush=True)
                        break
                    elif act == "pause":
                        manual_paused = True
                        chunk = None
                        print("[DEPLOY] ⏸ 已暂停模型推理（可随时恢复或用 SpaceMouse 接管）", flush=True)
                    elif act == "resume":
                        manual_paused = False
                        chunk = None
                        state_doc["mode"] = "policy"
                        print("[DEPLOY] ▶ 已恢复模型推理", flush=True)
                    elif act == "toggle_pause":
                        manual_paused = not manual_paused
                        chunk = None
                        print(f"[DEPLOY] 切换暂停状态 -> paused={manual_paused}", flush=True)
                    if "enable_hil" in cmd:
                        state_doc["hil_enabled"] = bool(cmd["enable_hil"])
                    if "auto_resume" in cmd:
                        state_doc["auto_resume"] = bool(cmd["auto_resume"])
                    if "idle_resume_sec" in cmd:
                        state_doc["idle_resume_sec"] = float(cmd["idle_resume_sec"])
                    if "record_mode" in cmd:
                        state_doc["record_mode"] = str(cmd["record_mode"])
                except Exception:
                    pass

            obs_t0 = time.perf_counter()
            obs = robot.get_observation()
            obs_ms = round((time.perf_counter() - obs_t0) * 1000.0, 1)
            obs_times.append(time.perf_counter())
            actual_hz = 0.0
            if len(obs_times) >= 2 and (obs_times[-1] - obs_times[0]) > 1e-6:
                actual_hz = round((len(obs_times) - 1) / (obs_times[-1] - obs_times[0]), 1)
            state_doc["capture_hz"] = actual_hz

            ext_img = obs["exterior_image"]
            wrist_img = obs["wrist_image"]

            if t0 - last_preview_t >= 0.12:
                last_preview_t = t0
                frame_seq += 1
                _save_jpeg(preview_dir / "exterior.jpg", ext_img)
                _save_jpeg(preview_dir / "wrist.jpg", wrist_img)
                _write_preview_meta(
                    preview_dir,
                    {
                        "updated_at": time.time(),
                        "frame_seq": frame_seq,
                        "source": f"deploy:{args.task}",
                        "mode": state_doc["mode"],
                        "step": state_doc["step"],
                        "capture_hz": actual_hz,
                        "target_fps": fps,
                        "width": cam_width,
                        "height": cam_height,
                        "exterior_ms": obs_ms,
                        "wrist_ms": obs_ms,
                        "message": f"部署运行中 ({state_doc['mode'].upper()}) · 频率 {actual_hz:.1f} Hz · 步数 {state_doc['step']}",
                    },
                )

            sm_active = False
            sm_action_dict: dict[str, float] | None = None
            if spacemouse is not None and state_doc["hil_enabled"]:
                try:
                    raw_axes, raw_buttons = spacemouse._expert.get_action()
                    axis_mag = float(np.linalg.norm(raw_axes[:6])) if len(raw_axes) >= 6 else 0.0
                    btn_pressed = any(int(b) != 0 for b in raw_buttons[:2]) if raw_buttons else False
                    if axis_mag > float(args.spacemouse_deadzone) or btn_pressed:
                        sm_active = True
                        last_spacemouse_active_t = t0
                        sm_action_dict = spacemouse.get_observations()
                except Exception:
                    pass

            prev_mode = state_doc["mode"]
            if sm_active:
                active_mode = "hil"
                if prev_mode != "hil":
                    chunk = None
                    print("[DEPLOY] 🕹️ 检测到 SpaceMouse 移动，已自动打断推理并进入 HIL 遥操作录制模式！", flush=True)
            elif prev_mode == "hil":
                idle_elapsed = t0 - last_spacemouse_active_t
                if state_doc["auto_resume"] and not manual_paused and idle_elapsed >= float(state_doc["idle_resume_sec"]):
                    active_mode = "policy"
                    chunk = None
                    print(f"[DEPLOY] 🤖 SpaceMouse 已静止 {idle_elapsed:.1f}s，自动恢复模型推理！", flush=True)
                elif state_doc["auto_resume"] and not manual_paused:
                    active_mode = "hil"
                    sm_action_dict = spacemouse.get_observations() if spacemouse is not None else None
                else:
                    active_mode = "paused"
                    if prev_mode != "paused":
                        print("[DEPLOY] ⏸ SpaceMouse 已停止，当前处于等待手动恢复状态（点击『继续推理』或按键恢复）", flush=True)
            elif manual_paused:
                active_mode = "paused"
            else:
                active_mode = "policy"

            state_doc["mode"] = active_mode

            state_vec = [float(obs[f"joint_{i+1}.pos"]) for i in range(7)] + [
                float(obs["gripper_state_norm"]),
                float(obs["gripper_cmd_bin"]),
            ] + [float(obs[f"ee_pose.{ax}"]) for ax in ("x", "y", "z", "rx", "ry", "rz")]

            executed_action: list[float] | None = None

            if active_mode == "paused":
                publish_state(message=f"⏸ 已暂停推理 · 实时相机 {actual_hz:.1f} Hz（移动 SpaceMouse 可接管，或点击『继续推理』恢复）")
                time.sleep(0.04)
                continue

            elif active_mode == "hil" and sm_action_dict is not None:
                robot.send_action(sm_action_dict)
                executed_action = [
                    float(sm_action_dict.get(f"delta_ee_pose.{ax}", 0.0))
                    for ax in ("x", "y", "z", "rx", "ry", "rz")
                ] + [float(sm_action_dict.get("gripper_cmd_bin", obs["gripper_cmd_bin"]))]
                state_doc["hil_steps"] += 1
                publish_state(message=f"🕹️ HIL 人工接管录制中 ({actual_hz:.1f} Hz · 已录 HIL {state_doc['hil_steps']} 步)")

            elif active_mode == "policy":
                if chunk is None or chunk_idx >= open_loop_horizon:
                    req = {
                        "observation/exterior_image_1_left": np.asarray(ext_img, dtype=np.uint8),
                        "observation/wrist_image_left": np.asarray(wrist_img, dtype=np.uint8),
                        "observation/state": np.asarray(state_vec, dtype=np.float32),
                        "prompt": args.prompt,
                    }
                    resp = policy.infer(req)
                    chunk = np.asarray(resp["actions"], dtype=np.float64)
                    chunk_idx = 0

                act_7d = chunk[chunk_idx]
                chunk_idx += 1
                action_dict = {
                    "delta_ee_pose.x": float(act_7d[0]),
                    "delta_ee_pose.y": float(act_7d[1]),
                    "delta_ee_pose.z": float(act_7d[2]),
                    "delta_ee_pose.rx": float(act_7d[3]),
                    "delta_ee_pose.ry": float(act_7d[4]),
                    "delta_ee_pose.rz": float(act_7d[5]),
                    "gripper_cmd_bin": 1.0 if float(act_7d[6]) >= 0.5 else 0.0,
                }
                robot.send_action(action_dict)
                executed_action = [float(x) for x in act_7d[:6]] + [action_dict["gripper_cmd_bin"]]
                state_doc["policy_steps"] += 1
                publish_state(
                    message=f"🤖 模型自动推理中 ({actual_hz:.1f} Hz · Policy {state_doc['policy_steps']} 步 | HIL {state_doc['hil_steps']} 步)"
                )

            step_num = state_doc["step"]
            should_record = (
                rec_steps_file is not None
                and executed_action is not None
                and (
                    state_doc["record_mode"] == "full_and_hil"
                    or (state_doc["record_mode"] == "hil_only" and active_mode == "hil")
                )
            )
            if should_record and rec_frames_ext is not None and rec_frames_wrist is not None:
                _save_jpeg(rec_frames_ext / f"{step_num:06d}.jpg", ext_img, quality=88)
                _save_jpeg(rec_frames_wrist / f"{step_num:06d}.jpg", wrist_img, quality=88)
                rec_steps_file.write(
                    json.dumps(
                        {
                            "step": step_num,
                            "timestamp": time.time(),
                            "source": active_mode,
                            "capture_hz": actual_hz,
                            "state": state_vec,
                            "action": executed_action,
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )
                rec_steps_file.flush()

            state_doc["step"] = step_num + 1
            elapsed = time.perf_counter() - t0
            sleep_s = max(0.0, (1.0 / fps) - elapsed)
            if sleep_s > 0:
                time.sleep(sleep_s)

        publish_state(phase="stopped", message="部署已安全停止")
        return 0
    except Exception as exc:
        publish_state(phase="error", message=f"部署异常退出: {exc}")
        print(f"[DEPLOY] 错误: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if rec_steps_file is not None:
            try:
                rec_steps_file.close()
            except Exception:
                pass
        if spacemouse is not None:
            try:
                spacemouse._expert.close()
            except Exception:
                pass
        try:
            robot.disconnect()
        except Exception:
            pass


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Franka Web Bridge Helper")
    sub = p.add_subparsers(dest="subcmd", required=True)

    p_state = sub.add_parser("robot-state")
    p_state.add_argument("--teleop-root", default="/data2/yjd/workspace/Franka-Teleop")
    p_state.add_argument("--nuc-ip", default="192.168.50.10")
    p_state.add_argument("--nuc-port", type=int, default=4242)
    p_state.add_argument("--desk-port", type=int, default=8443)

    p_desk = sub.add_parser("desk-action")
    p_desk.add_argument("--action", required=True, choices=("unlock", "lock", "fci-on", "fci-off", "status"))
    p_desk.add_argument("--desk-port", type=int, default=8443)
    p_desk.add_argument("--runtime-dir", required=True)
    p_desk.add_argument("--username", default="")
    p_desk.add_argument("--password", default="")
    p_desk.add_argument("--save-credentials", action="store_true")

    p_snap = sub.add_parser("camera-snapshot")
    p_snap.add_argument("--teleop-root", default="/data2/yjd/workspace/Franka-Teleop")
    p_snap.add_argument("--preview-dir", required=True)

    p_stream = sub.add_parser("camera-stream")
    p_stream.add_argument("--teleop-root", default="/data2/yjd/workspace/Franka-Teleop")
    p_stream.add_argument("--preview-dir", required=True)
    p_stream.add_argument("--fps", type=int, default=30)
    p_stream.add_argument("--preview-fps", type=float, default=8.0)
    p_stream.add_argument("--stop-file", default="")

    p_ds = sub.add_parser("dataset-preview")
    p_ds.add_argument("--dataset-dir", required=True)
    p_ds.add_argument("--episode-index", type=int, default=0)
    p_ds.add_argument("--preview-dir", required=True)

    p_rec = sub.add_parser("record-wrapper")
    p_rec.add_argument("--teleop-root", default="/data2/yjd/workspace/Franka-Teleop")
    p_rec.add_argument("--config-file", required=True)
    p_rec.add_argument("--storage-root", default="")
    p_rec.add_argument("--episodes", type=int, default=0)
    p_rec.add_argument("--preview-dir", required=True)
    p_rec.add_argument("--control-file", required=True)

    p_hil = sub.add_parser("hil-rollout")
    p_hil.add_argument("--teleop-root", default="/data2/yjd/workspace/Franka-Teleop")
    p_hil.add_argument("--deploy-root", required=True)
    p_hil.add_argument("--task", required=True)
    p_hil.add_argument("--prompt", required=True)
    p_hil.add_argument("--checkpoint", required=True)
    p_hil.add_argument("--port", type=int, required=True)
    p_hil.add_argument("--nuc-ip", default="192.168.50.10")
    p_hil.add_argument("--exterior-cam-id", default="CPC936300041")
    p_hil.add_argument("--wrist-cam-id", default="CPC93630001C")
    p_hil.add_argument("--cam-width", type=int, default=424)
    p_hil.add_argument("--cam-height", type=int, default=240)
    p_hil.add_argument("--fps", type=int, default=30)
    p_hil.add_argument("--open-loop-horizon", type=int, default=8)
    p_hil.add_argument("--enable-hil", type=int, default=1)
    p_hil.add_argument("--auto-resume", type=int, default=1)
    p_hil.add_argument("--idle-resume-sec", type=float, default=1.2)
    p_hil.add_argument("--spacemouse-deadzone", type=float, default=0.15)
    p_hil.add_argument("--record-mode", default="hil_only", choices=("none", "hil_only", "full_and_hil"))
    p_hil.add_argument("--record-dir", default="")
    p_hil.add_argument("--skip-reset", action="store_true")
    p_hil.add_argument("--preview-dir", required=True)
    p_hil.add_argument("--control-file", required=True)
    p_hil.add_argument("--status-file", required=True)

    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.subcmd == "robot-state":
        return cmd_robot_state(args)
    if args.subcmd == "desk-action":
        return cmd_desk_action(args)
    if args.subcmd == "camera-snapshot":
        return cmd_camera_snapshot(args)
    if args.subcmd == "camera-stream":
        return cmd_camera_stream(args)
    if args.subcmd == "dataset-preview":
        return cmd_dataset_episode_preview(args)
    if args.subcmd == "record-wrapper":
        return cmd_record_wrapper(args)
    if args.subcmd == "hil-rollout":
        return cmd_hil_rollout(args)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
