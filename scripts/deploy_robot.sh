#!/bin/bash
# Deploy the G1 web console to the robot dev PC and prepare the environment
# (first run installs tmux + ttyd). Run from your workstation:
#
#   ./scripts/deploy_robot.sh [robot-user@robot-ip]
#
# Defaults: unitree@10.33.32.237 (campus WiFi; wired fallback 192.168.123.164)
set -euo pipefail

REMOTE="${1:-unitree@10.33.32.237}"
DIR="$(cd "$(dirname "$0")/.." && pwd)"
TTYD_VERSION=1.7.7

echo "==> rsync project to ${REMOTE}:/home/unitree/g1web/app"
rsync -az --delete --exclude bin/ --exclude __pycache__ --exclude .git \
  "$DIR/" "$REMOTE:/home/unitree/g1web/app/"

echo "==> ensure venv + deps + tmux + ttyd on robot"
ssh "$REMOTE" bash -s <<'REMOTE_SETUP'
set -e
if [ ! -x /home/unitree/g1web/venv/bin/python ]; then
  echo 123 | sudo -S apt-get install -y -qq portaudio19-dev python3-venv python3-dev >/dev/null
  python3 -m venv /home/unitree/g1web/venv
  /home/unitree/g1web/venv/bin/pip install -q -U "pip<25" wheel setuptools
  /home/unitree/g1web/venv/bin/pip install -q unitree_webrtc_connect
fi
command -v tmux >/dev/null || echo 123 | sudo -S apt-get install -y -qq tmux
mkdir -p /home/unitree/g1web/app/bin
if [ ! -x /home/unitree/g1web/app/bin/ttyd ]; then
  echo "downloading ttyd (aarch64)…"
  curl -fsSL -o /tmp/ttyd "https://github.com/tsl0922/ttyd/releases/download/${TTYD_VER:-1.7.7}/ttyd.aarch64"
  chmod +x /tmp/ttyd && mv /tmp/ttyd /home/unitree/g1web/app/bin/ttyd
fi
/home/unitree/g1web/app/bin/ttyd --version || true
REMOTE_SETUP

echo "==> launch TUI in tmux + ttyd"
ssh "$REMOTE" "cd /home/unitree/g1web/app && bash scripts/run_tui_tmux.sh"
echo "deploy done."
