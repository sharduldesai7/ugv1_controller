"""Manual-drive state for the /drive page. No ROS imports, so it can be tested alone.

Rules:
  - Drive Enable is held by one client (browser tab) at a time.
  - The holder must keep contacting the server; if it goes quiet for
    ``link_timeout`` seconds, Drive Enable is released (reset to off).
  - Motion commands are normalised (-1..1) and scaled by the speed limits.
    A command older than ``cmd_timeout`` counts as zero.
  - Drive Enable can only be switched off when no motion command has been
    active for ``still_window`` seconds AND the IMU shows the rover is still
    over the same window. With no IMU data it refuses to switch off.
  - While enabled, tick() publishes the applied command at the caller's rate
    (zeros included). When disabled, nothing is published, so another source
    (such as Nav2) can own /cmd_vel.
"""

import threading
import time
from collections import deque

MIN_SPEED = 0.05
MAX_SPEED = 5.0
SPEED_STEP = 1.1

DISABLE_REASONS = {
    'moving': 'Stop the rover before switching off.',
    'settling': 'Wait for the rover to come to rest.',
    'imu_missing': 'No IMU data, cannot confirm the rover is still.',
    'imu_moving': 'The IMU still shows motion. Wait for it to settle.',
}


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DriveController:

    def __init__(self, publish, clock=time.monotonic,
                 lin=0.5, turn=1.0,
                 cmd_timeout=0.3, link_timeout=2.0,
                 still_window=0.5, still_gyro_range=0.08,
                 still_accel_range=0.5):
        self._publish = publish          # publish(linear_x, angular_z)
        self._clock = clock
        self._lock = threading.Lock()

        self.lin = lin
        self.turn = turn
        self.cmd_timeout = cmd_timeout
        self.link_timeout = link_timeout
        self.still_window = still_window
        self.still_gyro_range = still_gyro_range
        self.still_accel_range = still_accel_range

        self._driver = None              # client id holding Drive Enable
        self._last_seen = 0.0
        self._cmd = (0.0, 0.0)           # normalised x, th
        self._cmd_time = 0.0
        self._last_motion = None         # last time a non-zero command was applied
        self._publishing = False
        self._imu = deque()              # (t, gx, gy, gz, ax, ay, az)

    # ── helpers (call with the lock held) ────────────────────────────────────

    def _applied(self, now):
        """Return (x, th) normalised, or zeros if the command is stale."""
        if now - self._cmd_time > self.cmd_timeout:
            return 0.0, 0.0
        return self._cmd

    def _release(self, now):
        self._driver = None
        self._cmd = (0.0, 0.0)
        self._cmd_time = now

    def _imu_ranges(self, now):
        """Return (gyro_range, accel_range, span, age) over the still window."""
        cutoff = now - self.still_window
        samples = [s for s in self._imu if s[0] >= cutoff]
        if not self._imu:
            return None, None, 0.0, None
        age = now - self._imu[-1][0]
        if len(samples) < 2:
            return None, None, 0.0, age
        span = samples[-1][0] - samples[0][0]
        g = max(max(s[i] for s in samples) - min(s[i] for s in samples)
                for i in (1, 2, 3))
        a = max(max(s[i] for s in samples) - min(s[i] for s in samples)
                for i in (4, 5, 6))
        return g, a, span, age

    def _disable_check(self, now):
        """Return None if it is safe to switch off, else a reason key."""
        x, th = self._applied(now)
        if x != 0.0 or th != 0.0:
            return 'moving'
        if self._last_motion is not None and \
                now - self._last_motion < self.still_window:
            return 'settling'
        g, a, span, age = self._imu_ranges(now)
        if g is None or age is None or age > self.still_window \
                or span < 0.6 * self.still_window:
            return 'imu_missing'
        if g > self.still_gyro_range or a > self.still_accel_range:
            return 'imu_moving'
        return None

    # ── IMU input ────────────────────────────────────────────────────────────

    def imu_sample(self, gx, gy, gz, ax, ay, az):
        now = self._clock()
        with self._lock:
            self._imu.append((now, gx, gy, gz, ax, ay, az))
            while self._imu and now - self._imu[0][0] > 2.0:
                self._imu.popleft()

    # ── Client actions: each returns (ok, reason_key) ────────────────────────

    def enable(self, client):
        now = self._clock()
        with self._lock:
            if self._driver not in (None, client):
                return False, 'locked'
            self._driver = client
            self._last_seen = now
            self._cmd = (0.0, 0.0)
            self._cmd_time = now
            self._last_motion = None
            return True, None

    def disable(self, client):
        now = self._clock()
        with self._lock:
            if self._driver != client:
                return False, 'not_driver'
            reason = self._disable_check(now)
            if reason:
                self._last_seen = now
                return False, reason
            self._release(now)
            return True, None

    def command(self, client, x, th):
        now = self._clock()
        with self._lock:
            if self._driver != client:
                return False, 'not_driver'
            x = _clamp(float(x), -1.0, 1.0)
            th = _clamp(float(th), -1.0, 1.0)
            self._cmd = (x, th)
            self._cmd_time = now
            self._last_seen = now
            if x != 0.0 or th != 0.0:
                self._last_motion = now
            return True, None

    def adjust_speed(self, client, target, direction):
        now = self._clock()
        with self._lock:
            if self._driver != client:
                return False, 'not_driver'
            f = SPEED_STEP if direction > 0 else 1.0 / SPEED_STEP
            if target in ('both', 'lin'):
                self.lin = _clamp(self.lin * f, MIN_SPEED, MAX_SPEED)
            if target in ('both', 'turn'):
                self.turn = _clamp(self.turn * f, MIN_SPEED, MAX_SPEED)
            self._last_seen = now
            return True, None

    # ── Periodic work ────────────────────────────────────────────────────────

    def tick(self):
        """Call at about 20 Hz: drop a silent driver and publish the command."""
        now = self._clock()
        with self._lock:
            if self._driver is not None and \
                    now - self._last_seen > self.link_timeout:
                self._release(now)
            if self._driver is not None:
                x, th = self._applied(now)
                lin, ang = x * self.lin, th * self.turn
                if x != 0.0 or th != 0.0:
                    self._last_motion = now
                self._publishing = True
            elif self._publishing:
                lin, ang = 0.0, 0.0       # one last stop on release
                self._publishing = False
            else:
                return
        self._publish(lin, ang)

    # ── State for the page ───────────────────────────────────────────────────

    def snapshot(self, client):
        now = self._clock()
        with self._lock:
            if client and client == self._driver:
                self._last_seen = now        # polling counts as contact
            enabled = self._driver is not None
            you = enabled and client == self._driver
            x, th = self._applied(now)
            reason = self._disable_check(now) if you else None
            g, a, span, age = self._imu_ranges(now)
            return {
                'enabled': enabled,
                'you_drive': you,
                'locked': enabled and not you,
                'lin_max': round(self.lin, 3),
                'turn_max': round(self.turn, 3),
                'cmd_lin': round(x * self.lin, 3) if enabled else 0.0,
                'cmd_ang': round(th * self.turn, 3) if enabled else 0.0,
                'can_disable': you and reason is None,
                'disable_reason': reason,
                'disable_message': DISABLE_REASONS.get(reason),
                'imu_gyro_range': None if g is None else round(g, 4),
                'imu_accel_range': None if a is None else round(a, 4),
                'imu_age': None if age is None else round(age, 3),
            }
