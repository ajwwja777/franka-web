#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ -f runtime/web.pid ]]; then
  PID="$(cat runtime/web.pid || true)"
  if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
    kill -TERM "$PID" 2>/dev/null || true
    sleep 1
    kill -0 "$PID" 2>/dev/null && kill -KILL "$PID" 2>/dev/null || true
    echo "Stopped franka-web (PID $PID)"
  else
    echo "franka-web PID $PID not running"
  fi
  rm -f runtime/web.pid
else
  echo "No runtime/web.pid file found"
fi
