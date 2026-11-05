#!/usr/bin/env python3
"""G1 console TUI (phase 1): curses front-end over RobotLink.

Run inside tmux; pair with ttyd for browser access (scripts/run_tui_tmux.sh).
All control is dry-run unless the safety switch is armed — see safety.py.
"""

import argparse
import curses
import logging
import os
import threading
import time
from collections import deque

from .robotlink import RobotLink, SPORT_MODES
from .safety import SafetySwitch, ARM_CODE

ASCII_RAMP = " .:-=+*#%@"
SEND_HZ = 10.0
LOG_MAX = 400
MODE_MENU = [
    ("1", "walk", 500),
    ("2", "walk_waist", 501),
    ("3", "run (801)", 801),
    ("0", "stop_move", 1003),
    ("u", "stand_up", 1004),
    ("d", "stand_down", 1005),
]


class LogBus:
    def __init__(self, n=LOG_MAX):
        self.buf = deque(maxlen=n)
        self.cond = threading.Condition()
        self._last_throttle = {}

    def push(self, kind, msg):
        with self.cond:
            self.buf.append((time.strftime("%H:%M:%S"), kind, msg))
            self.cond.notify_all()

    def push_throttled(self, key, kind, msg, min_interval=1.0):
        now = time.time()
        if now - self._last_throttle.get(key, 0.0) >= min_interval:
            self._last_throttle[key] = now
            self.push(kind, msg)


def flatten(d, prefix="", depth=1):
    """Yield (key, scalar) for scalars, recursing one level into dicts."""
    for k, v in (d or {}).items():
        key = f"{prefix}{k}"
        if isinstance(v, (int, float, str, bool)):
            yield key, v
        elif isinstance(v, dict) and depth > 0:
            yield from flatten(v, key + ".", depth - 1)
        elif isinstance(v, list) and len(v) <= 8 and all(
                isinstance(x, (int, float)) for x in v):
            yield key, "[" + ", ".join(f"{x:.3g}" for x in v) + "]"


class TuiApp:
    def __init__(self, link: RobotLink, bus: LogBus):
        self.link = link
        self.bus = bus
        self.axes = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0}
        self.speed = 0.3
        self.show_joints = False
        self.last_send_t = 0.0
        self.overlay = None  # None | ("mode",) | ("arm", str)

    # ── drawing helpers ──────────────────────────────────────────────

    @staticmethod
    def _cattr(cpair=0, bold=False, reverse=False):
        a = curses.A_NORMAL
        if bold:
            a |= curses.A_BOLD
        if reverse:
            a |= curses.A_REVERSE
        return curses.color_pair(cpair) | a

    def draw(self, scr):
        scr.erase()
        h, w = scr.getmaxyx()
        if h < 20 or w < 80:
            try:
                scr.addstr(0, 0, f"terminal too small: {w}x{h} (need >=80x20)")
            except curses.error:
                pass
            return
        snap = self.link.snapshot()
        link, video, safety = snap["link"], snap["video"], snap["safety"]
        tele = snap["telemetry"]

        footer_h = 2
        body_h = h - 1 - footer_h
        left_w = min(44, max(30, w // 3))
        log_h = max(6, body_h // 3)
        tele_h = body_h - log_h

        self._draw_header(scr, 0, w, link, video, safety)
        self._panel(scr, 1, 0, tele_h, left_w, "telemetry")
        self._draw_telemetry(scr, 1, left_w, tele_h, tele, video)
        vid_x, vid_w = left_w - 1, w - left_w + 1
        self._panel(scr, 1, vid_x, tele_h, vid_w, "camera (v: toggle)")
        self._draw_video(scr, 1, vid_x, tele_h, vid_w)
        self._panel(scr, 1 + tele_h, 0, log_h, w, "log (l: clear)")
        self._draw_log(scr, 1 + tele_h, w, log_h - 2)
        self._draw_footer(scr, h - footer_h, w, safety)
        if self.overlay:
            self._draw_overlay(scr, h, w)

    @staticmethod
    def _panel(scr, y, x, h, w, title=""):
        """Draw a box frame directly on stdscr (no subwindows — they garble
        the screen when panels overlap)."""
        if h < 3 or w < 4:
            return
        try:
            scr.addch(y, x, curses.ACS_ULCORNER)
            scr.addch(y, x + w - 1, curses.ACS_URCORNER)
            scr.addch(y + h - 1, x, curses.ACS_LLCORNER)
            scr.hline(y, x + 1, curses.ACS_HLINE, w - 2)
            scr.hline(y + h - 1, x + 1, curses.ACS_HLINE, w - 2)
            scr.vline(y + 1, x, curses.ACS_VLINE, h - 2)
            scr.vline(y + 1, x + w - 1, curses.ACS_VLINE, h - 2)
            if y + h - 1 < scr.getmaxyx()[0] - 1:
                scr.addch(y + h - 1, x + w - 1, curses.ACS_LRCORNER)
            if title:
                scr.addstr(y, x + 2, f" {title} ", curses.A_BOLD)
        except curses.error:
            pass

    def _draw_header(self, scr, y, w, link, video, safety):
        st_color = {"connected": 2, "connecting": 3, "error": 1}.get(
            link["status"], 4)
        armed = safety["state"] == "ARMED"
        left = (f" G1 CONSOLE  [{link['status'].upper()}]"
                f"  ip={link['robot_ip'] or '-'}"
                f"  video={link['video_on'] and 'on' or 'off'}"
                f" {video['size']} {video['fps']}fps")[: w - 1]
        safety_txt = (f"SAFETY: {'ARMED' if armed else 'LOCKED'}"
                      f" (real ctl {'enabled' if safety['allow_real_control'] else 'DISABLED'})")
        if armed:
            safety_txt += f" [{safety['arm_seconds_left']}s]"
        try:
            scr.addstr(y, 0, left, self._cattr(st_color, bold=True, reverse=True))
            n = len(left)
            if n < w - 1:
                scr.addstr(y, n, " " * (w - 1 - n),
                           self._cattr(st_color, reverse=True))
            tail = safety_txt[: w - 1]
            scr.addstr(y, max(0, w - 1 - len(tail)), tail,
                       self._cattr(2 if armed else 1, bold=True, reverse=True))
        except curses.error:
            pass

    def _draw_telemetry(self, scr, y, width, height, tele, video):
        row = y + 1
        maxw = width - 3
        end = y + height - 1

        def put(text, color=0, bold=False):
            nonlocal row
            if row < end:
                try:
                    scr.addstr(row, 2, text[:maxw], self._cattr(color, bold))
                except curses.error:
                    pass
                row += 1

        put(f"video : {video['frames']} frames {video['size']} "
            f"{video['fps']}fps", 6 if video["frames"] else 0)
        row += 1

        sport = (tele.get("rt/lf/sportmodestate") or {}).get("data") or {}
        if sport:
            put("sport_mode_state:", bold=True)
            for k, v in flatten(sport):
                put(f"  {k} = {v}")
        else:
            put("sport_mode_state: (no data)", 4)
        row += 1

        low = (tele.get("rt/lf/lowstate") or {}).get("data") or {}
        if low:
            put("low_state:", bold=True)
            imu = low.get("imu_state") or {}
            rpy = imu.get("rpy") or [0, 0, 0]
            put(f"  imu.rpy = [{', '.join(f'{x:+.3f}' for x in rpy)}]")
            gyro = imu.get("gyroscope") or [0, 0, 0]
            put(f"  imu.gyro = [{', '.join(f'{x:+.3f}' for x in gyro)}]")
            put(f"  imu.temp = {imu.get('temperature')}")
            put(f"  mode_pr={low.get('mode_pr')} "
                f"mode_machine={low.get('mode_machine')}")
            motors = low.get("motor_state") or []
            active = sum(1 for m in motors if (m or {}).get("mode", 0) != 0)
            temps = [t for m in motors
                     for t in ((m or {}).get("temperature") or [])]
            put(f"  motors: {len(motors)} ({active} active) "
                f"maxT={max(temps) if temps else '-'}")
            if self.show_joints:
                for i in range(0, len(motors), 3):
                    cells = [f"J{j}:{(motors[j] or {}).get('q', 0):+.2f}"
                             for j in range(i, min(i + 3, len(motors)))]
                    put("  " + " ".join(cells))
        else:
            put("low_state: (no data)", 4)
        row += 1

        for topic in ("rt/lf/bmsstate", "rt/lf/secondary_imu"):
            slot = tele.get(topic) or {}
            data = slot.get("data")
            name = topic.split("/")[-1]
            if data:
                put(f"{name}:", bold=True)
                for k, v in list(flatten(data))[:8]:
                    put(f"  {k} = {v}")
            else:
                put(f"{name}: (no data)", 4)

        for topic, slot in tele.items():
            age = slot.get("age", 0)
            cnt = slot.get("count", 0)
            put(f"{topic.split('/')[-1]}: {cnt} msgs, age {age:.1f}s", 6
                if age < 3 else 4)

    def _draw_video(self, scr, y, x, height, width):
        small = None
        try:
            small = self.link.video.small
        except AttributeError:
            pass
        if small is None:
            try:
                scr.addstr(y + 2, x + 4, "no frames", self._cattr(4))
            except curses.error:
                pass
            return
        sh, sw = small.shape
        inner_y, inner_x = y + 1, x + 1
        inner_h, inner_w = height - 2, width - 2
        for cy in range(inner_h):
            sy = min(sh - 1, int(cy * sh / max(1, inner_h)))
            line = "".join(
                ASCII_RAMP[min(9, small[sy, min(sw - 1, int(cx * sw / max(1, inner_w)))] * 10 // 256)]
                for cx in range(inner_w))
            try:
                scr.addstr(inner_y + cy, inner_x, line)
            except curses.error:
                pass

    def _draw_log(self, scr, y, width, nrows):
        entries = list(self.bus.buf)[-nrows:]
        colors = {"real_publish": 2, "dry_run": 3, "error": 1,
                  "link": 6, "video": 5, "cmd": 7}
        for i, (ts, kind, msg) in enumerate(entries):
            try:
                scr.addstr(y + 1 + i, 2, f"{ts} {msg}"[:width - 3],
                           self._cattr(colors.get(kind, 0)))
            except curses.error:
                pass

    def _draw_footer(self, scr, y, w, safety):
        axes = self.axes
        try:
            scr.addstr(y, 0,
                       f" axes lx={axes['lx']:+.1f} ly={axes['ly']:+.1f} "
                       f"rx={axes['rx']:+.1f} ry={axes['ry']:+.1f} "
                       f"speed={self.speed:.1f} "
                       f"dry={safety['dry_runs']} real={safety['real_publishes']}"[: w - 1],
                       self._cattr(0, reverse=True))
            scr.addstr(y + 1, 0,
                       " WASD move  E/R turn  +/- speed  SPACE stop  P send  "
                       "M modes  V video  C conn  A arm  X disarm  J joints  "
                       "L clear  Q quit"[: w - 1],
                       self._cattr(0, reverse=True))
        except curses.error:
            pass

    def _draw_overlay(self, scr, h, w):
        if self.overlay[0] == "mode":
            lines = ["mode request (rt/api/sport/request):"] + [
                f"  [{k}] {name}" for k, name, _ in MODE_MENU]
            lines.append("  ESC cancel")
            self._box(scr, h, w, lines, 8)
        elif self.overlay[0] == "arm":
            txt = self.overlay[1]
            lines = [
                f"type {ARM_CODE} to arm real control ({self.link.safety.arm_window_s:.0f}s window)",
                f"> {txt}",
                "ESC cancel   real control must also be enabled via env",
            ]
            self._box(scr, h, w, lines, 9)

    @staticmethod
    def _box(scr, h, w, lines, color):
        bw = max(len(l) for l in lines) + 8
        bh = len(lines) + 4
        y0, x0 = max(0, (h - bh) // 2), max(0, (w - bw) // 2)
        # solid background then frame
        for r in range(y0, min(h - 1, y0 + bh)):
            try:
                scr.addstr(r, x0, " " * min(bw, w - x0))
            except curses.error:
                pass
        TuiApp._panel(scr, y0, x0, bh, bw)
        for i, l in enumerate(lines):
            try:
                scr.addstr(y0 + 2 + i, x0 + 4, l[: bw - 6],
                           curses.color_pair(color) | curses.A_BOLD)
            except curses.error:
                pass

    # ── key handling / command loop ──────────────────────────────────

    def handle_key(self, ch):
        if self.overlay:
            return self._handle_overlay_key(ch)
        ax = self.axes
        if ch in (ord("w"), ord("W")):
            ax["ly"] = min(1.0, ax["ly"] + self.speed)
        elif ch in (ord("s"), ord("S")):
            ax["ly"] = max(-1.0, ax["ly"] - self.speed)
        elif ch in (ord("a"), ord("A")):
            if ch == ord("A"):
                self.overlay = ("arm", "")
            else:
                ax["lx"] = max(-1.0, ax["lx"] - self.speed)
        elif ch in (ord("d"), ord("D")):
            ax["lx"] = min(1.0, ax["lx"] + self.speed)
        elif ch in (ord("q"), ord("Q")):
            return "quit"
        elif ch == ord("e"):
            ax["rx"] = min(1.0, ax["rx"] + self.speed)
        elif ch == ord("r"):
            ax["rx"] = max(-1.0, ax["rx"] - self.speed)
        elif ch in (ord("+"), ord("=")):
            self.speed = min(1.0, self.speed + 0.1)
        elif ch in (ord("-"), ord("_")):
            self.speed = max(0.1, self.speed - 0.1)
        elif ch == ord(" "):
            for k in ax:
                ax[k] = 0.0
            self.link.estop()
        elif ch in (ord("p"), ord("P")):
            self.link.send_velocity(ax["lx"], ax["ly"], ax["rx"], ax["ry"])
        elif ch in (ord("m"), ord("M")):
            self.overlay = ("mode",)
        elif ch in (ord("v"), ord("V")):
            self.link.set_video(not self.link.snapshot()["link"]["video_on"])
        elif ch in (ord("c"), ord("C")):
            if self.link.snapshot()["link"]["status"] == "connected":
                self.link.disconnect()
            else:
                self.link.start()
        elif ch in (ord("j"), ord("J")):
            self.show_joints = not self.show_joints
        elif ch in (ord("l"), ord("L")):
            self.bus.buf.clear()
        elif ch == ord("x") or ch == ord("X"):
            self.link.safety.disarm()
            self.bus.push("link", "disarmed")
        return None

    def _handle_overlay_key(self, ch):
        kind = self.overlay[0]
        if ch == 27:  # ESC
            self.overlay = None
        elif kind == "mode":
            for k, name, api in MODE_MENU:
                if ch == ord(k):
                    self.overlay = None
                    self.bus.push("cmd", f"mode request -> {name}")
                    self.link.send_sport_request(api)
                    return None
        elif kind == "arm":
            txt = self.overlay[1]
            if ch in (curses.KEY_BACKSPACE, 127, 8):
                txt = txt[:-1]
            elif ch in (10, curses.KEY_ENTER):
                ok = self.link.safety.arm(txt)
                self.bus.push("link", "ARMED" if ok else
                              "arm failed (check code / env / window)")
                self.overlay = None
                return None
            elif 32 <= ch < 127:
                txt += chr(ch)
            self.overlay = ("arm", txt)
        return None

    def send_loop(self):
        """Resend velocity at SEND_HZ while any axis is non-zero."""
        now = time.time()
        if now - self.last_send_t < 1.0 / SEND_HZ:
            return
        if any(v != 0.0 for v in self.axes.values()):
            self.last_send_t = now
            self.link.send_velocity(
                self.axes["lx"], self.axes["ly"], self.axes["rx"], self.axes["ry"])


def _silence_lib_prints():
    """The upstream lib print()s status lines straight to stdout, which
    would garble the curses UI — replace every module-level binding."""
    import unitree_webrtc_connect.webrtc_driver as drv
    import unitree_webrtc_connect.unitree_auth as auth
    import unitree_webrtc_connect.webrtc_datachannel as dc
    import unitree_webrtc_connect.webrtc_video as vid
    quiet = lambda *a, **k: None  # noqa: E731
    for mod in (drv, auth, dc, vid):
        if hasattr(mod, "print_status"):
            mod.print_status = quiet
    logging.getLogger("aiortc").setLevel(logging.CRITICAL)


def main(stdscr, args):
    # Library code print()s straight to stdout, which corrupts the curses
    # incremental redraw. Redirect the *Python-level* stdout to a file;
    # ncurses renders through its own C-level fd and is unaffected.
    import sys as _sys
    _stdout_log = open(os.environ.get("G1_TUI_STDOUT_LOG",
                                      "/tmp/g1tui_stdout.log"), "a", buffering=1)
    _sys.stdout = _stdout_log
    logging.basicConfig(level=logging.FATAL, stream=_stdout_log)
    _silence_lib_prints()
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    for i in range(1, 8):
        curses.init_pair(i, i, -1)

    bus = LogBus()
    safety = SafetySwitch()
    link = RobotLink(
        ip=args.ip, aes_key=args.aes_key, safety=safety,
        on_event=lambda kind, msg: bus.push(kind, msg))
    app = TuiApp(link, bus)

    stdscr.nodelay(True)
    stdscr.timeout(50)
    link.set_video(True)
    link.start()
    bus.push("link", f"TUI started, robot={args.ip}, "
                     f"real control {'ENABLED' if safety.allow_real_control else 'disabled'}")

    try:
        while True:
            ch = stdscr.getch()
            while ch != -1:
                if curses.KEY_RESIZE == ch:
                    curses.curs_set(0)
                else:
                    if app.handle_key(ch) == "quit":
                        return
                ch = stdscr.getch()
            app.send_loop()
            app.draw(stdscr)
            curses.doupdate()
    finally:
        link.shutdown()
        bus.push("link", "TUI stopped")


def cli():
    p = argparse.ArgumentParser(description="G1 console TUI")
    p.add_argument("--ip", default=os.environ.get("G1_ROBOT_IP",
                                                  "192.168.123.161"))
    p.add_argument("--aes-key", default=os.environ.get("G1_AES_128_KEY"))
    args = p.parse_args()
    curses.wrapper(main, args)


if __name__ == "__main__":
    cli()
