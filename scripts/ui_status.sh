#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PORT="${FRANKA_WEB_PORT:-8016}"
if [[ -f runtime/web.pid ]]; then
  PID="$(cat runtime/web.pid || true)"
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    echo "franka-web RUNNING (PID $PID) -> http://127.0.0.1:$PORT"
    exit 0
  fi
fi
echo "franka-web STOPPED"
exit 1
