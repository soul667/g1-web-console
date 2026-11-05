"""RobotLink: wraps unitree_webrtc_connect for the G1 console.

One background thread runs the asyncio loop (WebRTC peer + subscriptions +
video frame pump). The TUI/web layer talks to this object from any thread:

- ``snapshot()`` — telemetry/video/connection snapshot (plain dict)
- ``send_velocity`` / ``send_sport_request`` / ``stop`` — safety-gated control
- ``connect()`` / ``disconnect()`` — lifecycle

Real publishing happens ONLY through :meth:`_publish_guarded`, which checks
the :class:`~g1_console.safety.SafetySwitch` first.
"""

import asyncio
import json
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)

from .safety import SafetySwitch

log = logging.getLogger("g1console")

TELEMETRY_TOPICS = [
    "rt/lf/sportmodestate",
    "rt/lf/lowstate",
    "rt/lf/bmsstate",
    "rt/lf/secondary_imu",
]

# G1 humanoid control = one FSM on rt/api/sport/request.
# SET_FSM_ID (7101) + parameter {"data": <fsm_id>} switches state;
# the robot echoes the active id back as `fsm_id` in rt/lf/sportmodestate.
# (Verified against upstream examples: 500 Walk, 501 Walk+waist, 801 Run.
# Go2 SPORT_CMD ids like 1003/1004 do NOT apply to the humanoids.)
LOCO_SET_FSM_ID = 7101
LOCO_GET_FSM_ID = 7001
LOCO_SET_VELOCITY = 7105  # {"velocity": [vx, vy, omega], "duration": s}

G1_FSM = {
    "walk": 500,
    "walk_waist": 501,
    "run": 801,
}

VIDEO_ASCII_W = 96   # chars
VIDEO_ASCII_H = 48   # chars (before 2:1 aspect squeeze)


def _parse_payload(message) -> Optional[dict]:
    data = message.get("data") if isinstance(message, dict) else None
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except (ValueError, TypeError):
            return None
    return data if isinstance(data, dict) else None


@dataclass
class TelemetrySlot:
    count: int = 0
    last_ts: float = 0.0
    data: Optional[dict] = None


@dataclass
class VideoStats:
    frames: int = 0
    width: int = 0
    height: int = 0
    fps: float = 0.0
    last_ts: float = 0.0
    # small grayscale preview for ASCII rendering
    small: Optional[object] = None  # numpy (h, w) uint8


@dataclass
class LinkState:
    status: str = "disconnected"  # disconnected|connecting|connected|error
    error: str = ""
    robot_ip: str = ""
    signaling: str = ""
    connected_since: float = 0.0


class RobotLink:
    """Thread-safe façade over the WebRTC connection."""

    def __init__(self, ip: str, aes_key: Optional[str] = None,
                 safety: Optional[SafetySwitch] = None,
                 auto_reconnect: bool = True,
                 on_event: Optional[Callable[[str, str], None]] = None,
                 ascii_preview: bool = True):
        self.ip = ip
        self.aes_key = aes_key
        self.safety = safety or SafetySwitch()
        self.auto_reconnect = auto_reconnect
        self.on_event = on_event or (lambda kind, msg: None)
        self.ascii_preview = ascii_preview

        self.state = LinkState(robot_ip=ip)
        self.telemetry = {t: TelemetrySlot() for t in TELEMETRY_TOPICS}
        self.video = VideoStats()

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._conn: Optional[UnitreeWebRTCConnection] = None
        self._video_on = False
        self._stop_flag = threading.Event()
        self._loop_ready = threading.Event()
        self._lock = threading.Lock()
        self._fps_ema = 0.0

    # ── lifecycle ────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the background loop thread and connect."""
        if self._thread and self._thread.is_alive():
            self._call_async(self._connect())
            return
        self._stop_flag.clear()
        self._loop_ready = threading.Event()
        self._thread = threading.Thread(
            target=self._run_loop, name="g1-robotlink", daemon=True)
        self._thread.start()
        if not self._loop_ready.wait(timeout=10):
            raise RuntimeError("robotlink loop thread failed to start")
        self._call_async(self._connect())

    def shutdown(self) -> None:
        """Tear down connection and stop the background thread."""
        self._stop_flag.set()
        self._video_on = False
        fut = self._call_async(self._teardown())
        if fut:
            try:
                fut.result(timeout=5)
            except Exception:
                pass
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=5)

    def disconnect(self) -> None:
        """Drop the WebRTC connection but keep the loop thread alive."""
        self._call_async(self._teardown())

    def reconnect(self) -> None:
        self._call_async(self._reconnect())

    def set_video(self, on: bool) -> None:
        self._video_on = on
        self._call_async(self._apply_video())

    # ── snapshot for UIs ─────────────────────────────────────────────

    def snapshot(self) -> dict:
        with self._lock:
            tele = {
                t: {"count": s.count, "age": max(0.0, time.time() - s.last_ts),
                    "data": s.data}
                for t, s in self.telemetry.items()
            }
            state = {
                "status": self.state.status,
                "error": self.state.error,
                "robot_ip": self.state.robot_ip,
                "connected_since": self.state.connected_since,
                "video_on": self._video_on,
            }
            video = {
                "frames": self.video.frames,
                "size": (f"{self.video.width}x{self.video.height}"
                         if self.video.width else ""),
                "fps": round(self.video.fps, 1),
            }
        return {"link": state, "telemetry": tele, "video": video,
                "safety": self._safety_view()}

    def _safety_view(self) -> dict:
        s = self.safety
        return {
            "allow_real_control": s.allow_real_control,
            "state": s.state,
            "arm_seconds_left": round(s.seconds_left(), 1),
            "real_publishes": s.real_publishes,
            "dry_runs": s.dry_runs,
            "rejected": s.rejected,
        }

    # ── control (all safety-gated) ───────────────────────────────────

    def send_velocity(self, lx: float, ly: float, rx: float, ry: float,
                      keys: int = 0) -> dict:
        """Velocity command on rt/wirelesscontroller (Unitree app semantics:
        ly = forward/back, lx = lateral, rx = turn, ry = spare)."""
        return self._publish_guarded(
            "velocity",
            lambda: self._conn.datachannel.pub_sub.publish_without_callback(
                "rt/wirelesscontroller",
                {"lx": lx, "ly": ly, "rx": rx, "ry": ry, "keys": keys},
            ),
            detail=f"lx={lx:+.2f} ly={ly:+.2f} rx={rx:+.2f} ry={ry:+.2f}")

    def send_sport_request(self, api_id: int, data=None) -> dict:
        """RPC on rt/api/sport/request (mode switches etc.)."""
        payload = {"api_id": api_id}
        if data is not None:
            payload["parameter"] = {"data": data}

        async def _do():
            return await self._conn.datachannel.pub_sub.publish_request_new(
                "rt/api/sport/request", payload)

        return self._publish_guarded(
            "sport_request", lambda: self._call_async(_do()).result(timeout=10),
            detail=f"api_id={api_id} data={data}")

    def set_fsm(self, fsm_id: int) -> dict:
        """Switch the G1 FSM state (SET_FSM_ID 7101)."""
        return self.send_sport_request(LOCO_SET_FSM_ID, fsm_id)

    def get_fsm(self) -> dict:
        """Query the active FSM id (GET_FSM_ID 7001)."""
        return self.send_sport_request(LOCO_GET_FSM_ID)

    def estop(self) -> dict:
        """STOP: zero the virtual joystick. (Velocity axes are what the
        walk FSM consumes; there is no Go2-style StopMove api on G1.)"""
        return self.send_velocity(0, 0, 0, 0)

    def _publish_guarded(self, action: str, publish, detail: str = "") -> dict:
        decision = self.safety.guard(action)
        if decision["allowed"]:
            try:
                publish()
                self.on_event("real_publish", f"REAL {action} {detail}")
                return decision
            except Exception as e:  # noqa: BLE001
                self.on_event("error", f"publish failed: {e}")
                return {"allowed": False, "mode": "error", "reason": str(e)}
        self.on_event("dry_run", f"DRY-RUN {action} {detail} "
                                 f"({decision['reason']})")
        return decision

    # ── background asyncio machinery ─────────────────────────────────

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop_ready.set()
        try:
            self._loop.run_until_complete(self._watchdog())
        except RuntimeError:
            pass  # loop stopped while watchdog pending — normal shutdown
        except Exception:  # noqa: BLE001
            log.exception("robotlink loop crashed")
        finally:
            self._loop.close()

    def _call_async(self, coro):
        if not self._loop or self._loop.is_closed():
            return None
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    async def _watchdog(self) -> None:
        backoff = 1.0
        while not self._stop_flag.is_set():
            await asyncio.sleep(1.0)
            if not self.auto_reconnect:
                continue
            if self._conn is not None and not self._conn.isConnected:
                self.on_event("link", "connection lost, reconnecting…")
                try:
                    await self._teardown()
                    await self._connect()
                    backoff = 1.0
                except Exception as e:  # noqa: BLE001
                    self.on_event("error", f"reconnect failed: {e}")
                    await asyncio.sleep(backoff)
                    backoff = min(backoff * 2, 30.0)

    async def _connect(self) -> None:
        with self._lock:
            self.state.status = "connecting"
            self.state.error = ""
        self.on_event("link", f"connecting to {self.ip} …")
        conn = UnitreeWebRTCConnection(
            WebRTCConnectionMethod.LocalSTA,
            ip=self.ip,
            aes_128_key=self.aes_key,
            device_type="G1",
        )
        await conn.connect()
        conn.video.add_track_callback(self._video_track_handler)
        for topic in TELEMETRY_TOPICS:
            conn.datachannel.pub_sub.subscribe(topic, self._make_telem_cb(topic))
        self._conn = conn
        with self._lock:
            self.state.status = "connected"
            self.state.connected_since = time.time()
        self.on_event("link", f"connected to {self.ip}")
        if self._video_on:
            await self._apply_video()

    async def _reconnect(self) -> None:
        await self._teardown()
        await self._connect()

    async def _teardown(self) -> None:
        if self._conn is not None:
            try:
                await self._conn.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._conn = None
        with self._lock:
            if self.state.status != "error":
                self.state.status = "disconnected"

    async def _apply_video(self) -> None:
        if self._conn is not None:
            try:
                self._conn.video.switchVideoChannel(self._video_on)
                self.on_event("video",
                              f"video channel {'on' if self._video_on else 'off'}")
            except Exception as e:  # noqa: BLE001
                self.on_event("error", f"video switch failed: {e}")

    def _make_telem_cb(self, topic: str):
        def cb(message):
            data = _parse_payload(message)
            with self._lock:
                slot = self.telemetry[topic]
                slot.count += 1
                slot.last_ts = time.time()
                if data is not None:
                    slot.data = data
        return cb

    async def _video_track_handler(self, track) -> None:
        import cv2
        import numpy as np
        self.on_event("video", f"track received kind={track.kind}")
        try:
            while True:
                frame = await track.recv()
                now = time.time()
                dt = now - self.video.last_ts if self.video.last_ts else 0.1
                self._fps_ema = (self._fps_ema * 0.8 +
                                 (1.0 / max(dt, 1e-3)) * 0.2) if dt < 5 else 0.0
                small = (cv2.resize(
                    frame.to_ndarray(format="gray"),
                    (VIDEO_ASCII_W, VIDEO_ASCII_H // 2),
                    interpolation=cv2.INTER_AREA,
                ) if self.ascii_preview and _has_cv2() else None)
                with self._lock:
                    self.video.frames += 1
                    self.video.width = frame.width
                    self.video.height = frame.height
                    self.video.fps = self._fps_ema
                    self.video.last_ts = now
                    self.video.small = small
        except Exception as e:  # noqa: BLE001
            self.on_event("video", f"track ended ({e})")


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False
