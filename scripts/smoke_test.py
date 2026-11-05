#!/usr/bin/env python3
"""Automated smoke test for the G1 console link layer.

Run ON the robot dev PC (needs access to the main board 192.168.123.161).

Checks — none of them move the robot:
 1. WebRTC connect (LocalSTA, legacy or key auth)
 2. video frames arrive (>=10 in window)
 3. telemetry topics stream (sportmodestate, bmsstate)
 4. velocity + sport commands are DRY-RUN while locked; real publish count == 0
 5. safety arm path unit-test: env-disabled cannot arm; enabled+code guards OK
 6. tmux session / ttyd port if the web terminal is up (soft checks)

Exit code 0 = all PASS.
"""

import asyncio
import json
import os
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from g1_console.robotlink import RobotLink  # noqa: E402
from g1_console.safety import SafetySwitch, ARM_CODE  # noqa: E402

ROBOT_IP = os.environ.get("G1_ROBOT_IP", "192.168.123.161")
WINDOW_S = 12

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))


def main():
    safety = SafetySwitch()
    events = []
    link = RobotLink(
        ip=ROBOT_IP, aes_key=os.environ.get("G1_AES_128_KEY"),
        safety=safety, on_event=lambda k, m: events.append((time.time(), k, m)))
    link.set_video(True)
    link.start()

    t0 = time.time()
    while time.time() - t0 < 30 and link.snapshot()["link"]["status"] != "connected":
        time.sleep(0.5)
    snap = link.snapshot()
    check("1. webrtc connect", snap["link"]["status"] == "connected",
          snap["link"].get("error", ""))

    t0 = time.time()
    while time.time() - t0 < WINDOW_S and link.video.frames < 10:
        time.sleep(0.5)
    snap = link.snapshot()
    check("2. video frames >= 10", snap["video"]["frames"] >= 10,
          f"frames={snap['video']['frames']} {snap['video']['size']} "
          f"{snap['video']['fps']}fps")

    tele = snap["telemetry"]
    check("3a. sportmodestate streaming",
          tele["rt/lf/sportmodestate"]["count"] >= 10,
          f"count={tele['rt/lf/sportmodestate']['count']}")
    check("3b. bmsstate streaming", tele["rt/lf/bmsstate"]["count"] >= 10,
          f"count={tele['rt/lf/bmsstate']['count']}")

    # 4. dry-run commands while locked
    v = link.send_velocity(0.0, 0.1, 0.0, 0.0)
    s = link.send_sport_request(1003)  # StopMove — still dry-run when locked
    check("4a. velocity command dry-run", v.get("mode") == "dry_run", str(v))
    check("4b. sport request dry-run", s.get("mode") == "dry_run", str(s))
    check("4c. zero real publishes", safety.real_publishes == 0,
          f"real={safety.real_publishes} dry={safety.dry_runs}")

    # 5. safety arm path unit test (guard() only — nothing published)
    sw = SafetySwitch()
    check("5a. arm rejected when env disabled", not sw.arm(ARM_CODE) and
          sw.state == "LOCKED")
    os.environ["G1_CONSOLE_ALLOW_REAL_CONTROL"] = "1"
    sw2 = SafetySwitch()
    armed = sw2.arm(ARM_CODE)
    decision = sw2.guard("test")  # guard does NOT publish by itself
    check("5b. arm+guard allows when enabled", armed and
          decision["allowed"] and sw2.state == "ARMED")
    sw2.disarm()
    check("5c. disarm relocks", sw2.guard("test")["allowed"] is False)
    del os.environ["G1_CONSOLE_ALLOW_REAL_CONTROL"]

    # 6. web terminal soft checks
    r = subprocess.run(["tmux", "has-session", "-t", "g1tui"],
                       capture_output=True)
    check("6a. tmux session g1tui", r.returncode == 0, "run scripts/run_tui_tmux.sh")
    sock = socket.socket()
    sock.settimeout(2)
    try:
        sock.connect(("127.0.0.1", int(os.environ.get("G1_TTYD_PORT", "7681"))))
        check("6b. ttyd port listening", True)
    except OSError as e:
        check("6b. ttyd port listening", False, str(e))
    finally:
        sock.close()

    link.shutdown()

    npass = sum(1 for _, ok, _ in results if ok)
    print(f"\n=== SMOKE {'PASS' if npass == len(results) else 'FAIL'}: "
          f"{npass}/{len(results)} ===")
    print(json.dumps({"events": len(events),
                      "real_publishes": safety.real_publishes},
                     indent=1))
    sys.exit(0 if npass == len(results) else 1)


if __name__ == "__main__":
    main()
