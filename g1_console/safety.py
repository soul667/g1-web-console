"""Safety lock for G1 console.

Real control is DISABLED by default. A command can only reach the robot when
BOTH conditions hold:

1. The server was started with ``G1_CONSOLE_ALLOW_REAL_CONTROL=1``.
2. The operator completed the arm sequence in the UI (state -> ARMED),
   and the arm window (default 60 s) has not expired.

Without (1) every command is a dry run: logged and counted, never published.
"""

import os
import time
import threading

ARM_CODE = "ARM-G1"
DEFAULT_ARM_WINDOW_S = 60.0

LOCKED = "LOCKED"
ARMED = "ARMED"


class SafetySwitch:
    def __init__(self, arm_window_s: float = DEFAULT_ARM_WINDOW_S):
        self.enabled = os.environ.get("G1_CONSOLE_ALLOW_REAL_CONTROL") == "1"
        self.arm_window_s = arm_window_s
        self._armed_until = 0.0
        self._lock = threading.Lock()
        self.real_publishes = 0
        self.dry_runs = 0
        self.rejected = 0

    @property
    def state(self) -> str:
        with self._lock:
            if self.enabled and time.time() < self._armed_until:
                return ARMED
            return LOCKED

    @property
    def allow_real_control(self) -> bool:
        return self.enabled

    def arm(self, code: str) -> bool:
        """Arm with the confirmation code. Returns success."""
        if not self.enabled:
            return False
        if code != ARM_CODE:
            return False
        with self._lock:
            self._armed_until = time.time() + self.arm_window_s
        return True

    def disarm(self) -> None:
        with self._lock:
            self._armed_until = 0.0

    def seconds_left(self) -> float:
        with self._lock:
            return max(0.0, self._armed_until - time.time())

    def guard(self, action: str) -> dict:
        """Decide whether *action* may be published for real.

        Returns ``{"allowed": bool, "mode": "real"|"dry_run", "reason": str}``.
        Callers MUST only publish when ``allowed`` is True.
        """
        state = self.state
        if state == ARMED:
            self.real_publishes += 1
            return {"allowed": True, "mode": "real", "reason": "armed"}
        if not self.enabled:
            self.dry_runs += 1
            return {
                "allowed": False,
                "mode": "dry_run",
                "reason": "real control disabled (G1_CONSOLE_ALLOW_REAL_CONTROL!=1)",
            }
        self.rejected += 1
        return {
            "allowed": False,
            "mode": "locked",
            "reason": "armed state required (arm sequence not completed/expired)",
        }
