"""Waypoint following for the simulation route (pure logic, no ROS)."""

import math

# World coordinates (metres); the rover spawns at (-6.0, -3.8) facing +x.
DEFAULT_ROUTE = [
    6.0, -3.8,      # along the south lane
    6.0, -1.2,
    -6.0, -1.2,     # west through the first aisle
    -6.0, 1.2,
    6.0, 1.2,       # east through the second aisle
    6.0, 3.8,
    -6.0, 3.8,      # west along the north lane
    -6.0, -3.8,     # back to the start
]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteFollower:
    """Pure logic: pose and time in, (linear, angular) out. No ROS."""

    def __init__(self, route, speed=0.3, turn=0.6, start_delay=5.0, pos_tol=0.08, yaw_tol=0.04):
        self.points = [(route[i], route[i + 1]) for i in range(0, len(route) - 1, 2)]
        self.speed, self.turn = speed, turn
        self.start_delay = start_delay
        self.pos_tol, self.yaw_tol = pos_tol, yaw_tol
        self.index = 0
        self.mode = 'wait'
        self.t0 = None

    @property
    def done(self):
        return self.mode == 'done'

    def step(self, t, x, y, yaw):
        if self.t0 is None:
            self.t0 = t
        if self.mode == 'wait':
            if t - self.t0 < self.start_delay:
                return 0.0, 0.0
            self.mode = 'turn'
        if self.mode == 'done':
            return 0.0, 0.0
        tx, ty = self.points[self.index]
        dist = math.hypot(tx - x, ty - y)
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        if self.mode == 'turn':
            if abs(err) < self.yaw_tol:
                self.mode = 'drive'
            else:
                w = max(0.15, min(self.turn, 1.5 * abs(err)))
                return 0.0, math.copysign(w, err)
        if dist < self.pos_tol:
            self.index += 1
            self.mode = 'done' if self.index >= len(self.points) else 'turn'
            return 0.0, 0.0
        if abs(err) > 0.5:
            self.mode = 'turn'
            return 0.0, 0.0
        lin = self.speed * min(1.0, max(0.25, dist / 0.4))
        return lin, max(-self.turn, min(self.turn, 1.5 * err))
