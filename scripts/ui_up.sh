#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PORT="${FRANKA_WEB_PORT:-8016}"
HOST="${FRANKA_WEB_HOST:-0.0.0.0}"
mkdir -p runtime/logs runtime/preview

if [[ -f runtime/web.pid ]]; then
  OLD_PID="$(cat runtime/web.pid || true)"
  if [[ -n "$OLD_PID" ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    echo "franka-web is already running (PID $OLD_PID) on http://127.0.0.1:$PORT"
    exit 0
  fi
fi

BIN="$ROOT/target/release/franka-web"
if [[ ! -x "$BIN" ]]; then
  BIN="$ROOT/bin/franka-web"
fi
if [[ ! -x "$BIN" ]]; then
  echo "Building franka-web release binary..."
  export PATH="$HOME/.cargo/bin:$PATH"
  cargo build --release --offline
  BIN="$ROOT/target/release/franka-web"
fi

nohup "$BIN" serve --host "$HOST" --port "$PORT" >> runtime/logs/web.log 2>&1 &
NEW_PID=$!
echo "$NEW_PID" > runtime/web.pid
sleep 1
if kill -0 "$NEW_PID" 2>/dev/null; then
  echo "franka-web started (PID $NEW_PID) -> http://127.0.0.1:$PORT"
else
  echo "franka-web failed to start; check runtime/logs/web.log" >&2
  exit 1
fi
