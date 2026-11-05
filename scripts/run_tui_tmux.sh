#!/bin/bash
# Launch the G1 console TUI inside tmux and serve it in the browser via ttyd.
#
# Usage (on the robot dev PC):
#   ./scripts/run_tui_tmux.sh            # start/restart
#   ./scripts/run_tui_tmux.sh stop       # stop everything
#
# Browser: http://<robot-ip>:7681/  (basic auth g1:<G1_WEB_PASSWORD, default g1console>)
# Real control stays disabled unless G1_CONSOLE_ALLOW_REAL_CONTROL=1 (see safety.py).
set -euo pipefail

SESSION=g1tui
TTYD_PORT="${G1_TTYD_PORT:-7681}"
WEB_USER="${G1_WEB_USER:-g1}"
WEB_PASS="${G1_WEB_PASS:-g1console}"
VENV=/home/unitree/g1web/venv
DIR="$(cd "$(dirname "$0")/.." && pwd)"
TTYD_BIN="${DIR}/bin/ttyd"

command -v tmux >/dev/null || { echo "tmux not installed: sudo apt install -y tmux"; exit 1; }

stop_all() {
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  pkill -f "ttyd.*${TTYD_PORT}" 2>/dev/null || true
  echo "stopped"
}

if [ "${1:-}" = "stop" ]; then stop_all; exit 0; fi
stop_all

# TUI inside a persistent tmux session (ttyd clients attach to it).
# stderr (ffmpeg/aiortc warnings) goes to a file so it can't garble the UI.
# Pass the safety env explicitly: the tmux server may not inherit this shell's
# environment. Real control needs BOTH this env =1 and the in-TUI ARM-G1 step.
tmux new-session -d -s "$SESSION" -x "${G1_TMUX_W:-160}" -y "${G1_TMUX_H:-44}" \
  "cd ${DIR} && G1_CONSOLE_ALLOW_REAL_CONTROL=${G1_CONSOLE_ALLOW_REAL_CONTROL:-0} \
${VENV}/bin/python -m g1_console.tui 2>/tmp/g1tui_stderr.log; echo 'TUI exited — press Enter to close'; read"
# -x/-y fixes the window size (manual); switch back to `latest` so a browser
# client attaching via ttyd resizes the window to fit (no dead dotted area).
tmux set-window-option -t "$SESSION" window-size latest 2>/dev/null || true

# ttyd: writable (interaction), basic auth, attaches to the same tmux session
if [ ! -x "$TTYD_BIN" ]; then
  echo "ttyd missing — run scripts/deploy_robot.sh first" >&2
  exit 1
fi
nohup "$TTYD_BIN" --port "$TTYD_PORT" --writable \
  --credential "${WEB_USER}:${WEB_PASS}" \
  tmux attach -t "$SESSION" >/tmp/ttyd_g1.log 2>&1 &

sleep 1
echo "TUI tmux session : $SESSION  (tmux attach -t $SESSION)"
echo "Web terminal     : http://$(hostname -I | awk '{print $1}'):${TTYD_PORT}/  (user: ${WEB_USER})"
grep -q . /tmp/ttyd_g1.log && { echo "ttyd log:"; tail -3 /tmp/ttyd_g1.log; } || true
