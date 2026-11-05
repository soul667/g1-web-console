# g1-web-console

Browser-based console for the **Unitree G1 humanoid** built on the
WebRTC protocol used by the official mobile app — reverse-engineered and
packaged by [`unitree_webrtc_connect`](https://github.com/legion1581/unitree_webrtc_connect).

**Phase 1 (this repo, working on a real G1):** a full-featured **TUI** running
inside `tmux`, served to any browser through `ttyd` — camera stream (rendered
as live ASCII), telemetry (FSM state, 29-joint lowstate, BMS battery, IMU) and
a **safety-locked** control path that logs every command as a dry run unless
explicitly armed. **Phase 2** (planned): a native web UI with H.264 WebRTC
video, virtual gamepad and telemetry cards on top of the same `RobotLink`
layer.

```
            WebRTC (app protocol)                browser
  G1 main ─────────────────────► robot dev PC ─────────────► you
  board :8081   video/telemetry  RobotLink TUI   ttyd/tmux   any device
  192.168.123.161                192.168.123.164  :7681
```

## Safety model — read this first

Real control is **disabled by default** and requires *both*:

1. the server env `G1_CONSOLE_ALLOW_REAL_CONTROL=1`, **and**
2. an arm sequence in the UI (`A`, then type `ARM-G1`; 60 s window, auto-relock).

Anything else → every command is a **dry run**: logged, counted, never
published. The smoke test asserts `real_publishes == 0` end-to-end.

> A `ttyd` shell is a shell on the robot: change the basic-auth password via
> `G1_WEB_USER`/`G1_WEB_PASS`, and do not expose port 7681 to untrusted
> networks.

## Quick start (on the robot's dev PC)

```bash
# 1) environment (once) — Ubuntu 20.04 / Python 3.8, ARMv8
sudo apt install -y portaudio19-dev python3-venv tmux
python3 -m venv ~/g1web/venv
~/g1web/venv/bin/pip install unitree_webrtc_connect

# 2) get the code
git clone https://github.com/soul667/g1-web-console.git ~/g1web/app
mkdir -p ~/g1web/app/bin
curl -fsSL -o ~/g1web/app/bin/ttyd \
  https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64
chmod +x ~/g1web/app/bin/ttyd

# 3) launch TUI in tmux + web terminal
cd ~/g1web/app && bash scripts/run_tui_tmux.sh
#    stop with:  bash scripts/run_tui_tmux.sh stop
```

Open `http://<dev-pc-ip>:7681/` (default credentials `g1` / `g1console`),
or attach locally with `tmux attach -t g1tui`.

Connection defaults: main board `192.168.123.161` (`G1_ROBOT_IP` override).
Firmware ≥ 1.5.1 speaks `data2=3` and additionally needs the per-device
AES key (`G1_AES_128_KEY`, see upstream docs); our G1 runs legacy auth and
connects without any key.

### Operating sequence (real control)

The G1 is an FSM: velocity axes only work **after** switching to a
locomotion state. Mode = `SET_FSM_ID` (api 7101) on `rt/api/sport/request`;
the active state echoes back as `fsm_id` in the telemetry panel.

```
1. start server with G1_CONSOLE_ALLOW_REAL_CONTROL=1   (run_tui_tmux.sh forwards it)
2. press A, type ARM-G1, Enter        → 60 s arm window (press A again to extend)
3. press M then 1                      → FSM 500 walk (robot stands & enters walk)
   (501 walk+waist, 801 run)
4. W A S D / E R                       → drive (auto-sent at 10 Hz while non-zero)
5. SPACE                               → zero the axes
6. X                                   → relock immediately
```

Watch `fsm_id` in the telemetry panel: `0` = zero-torque default, and WASD
does nothing until it reads 500/501/801.

### Keys

| key | action |
|-----|--------|
| `M` | mode menu (FSM 500/501/801) |
| `W A S D` | move (ly / lx), sticky axes |
| `E` / `R` | turn (rx) |
| `+` / `-` | speed scale |
| `Space` | STOP (zero the axes) |
| `P` | send current axes once |
| `V` | camera on/off |
| `J` | per-joint q values |
| `A` / `X` | arm / disarm real control |
| `C` | connect / disconnect |
| `L`, `Q` | clear log, quit |

## Smoke test (no robot motion)

```bash
~/g1web/venv/bin/python scripts/smoke_test.py
```

Verified on the real robot (2026-11-05): connect, 1280×720 ~15 fps video,
sport-state 15 Hz, BMS 20 Hz, dry-run commands with zero real publishes,
safety-lock state machine, tmux + ttyd web terminal — **12/12 PASS**.

## Layout

```
g1_console/
  robotlink.py   thread-safe façade over unitree_webrtc_connect:
                 telemetry cache, video pump, safety-gated publishing
  safety.py      two-key safety lock (env + arm window), dry-run counters
  tui.py         curses front-end (tmux + ttyd for browser access)
scripts/
  run_tui_tmux.sh   launch/restart/stop the stack
  deploy_robot.sh   rsync + env bootstrap from a workstation
  smoke_test.py     automated assertions, exit-code driven
```

## Roadmap

- [x] TUI: telemetry, ASCII camera, dry-run control, safety lock
- [x] Web access via ttyd/tmux (this is already "the web console" — phase 1)
- [ ] Phase 2 web UI: H.264 WebRTC to the browser, virtual gamepad,
      telemetry cards, arm confirmation UI (same `RobotLink` backend)
- [ ] ARM/G1 arm API topics (`rt/api/arm/request`), lidar point cloud view

## Credits & license

- Protocol layer: [`unitree_webrtc_connect`](https://github.com/legion1581/unitree_webrtc_connect) (MIT)
- Web terminal: [`ttyd`](https://github.com/tsl0922/ttyd) (MIT)
- This repo: MIT — see [LICENSE](LICENSE).
