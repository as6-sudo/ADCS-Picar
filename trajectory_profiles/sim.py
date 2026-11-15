"""ArenaSim: obs_avoidance_profile_new's SimCar extended with the arena
layout, the sliding obstacle (O0 -> O1), tape strips under the grayscale
module, a slewing steering servo and picarx's inner-wheel slowdown on arcs.
Like SimCar it is also its own clock, so a whole take runs in a fraction
of a second.

The simulated car's "truth" is close to, but deliberately not exactly, the
default calibration tables, so --sim exercises the timing model, the tape
stops and the sensor stops the way the real car would (only better behaved):
  * ground speed per wheel ~ 70 * (PWM - 35) / 65 cm/s, PWM = 50 + s/2 (int);
    with steering, the inner rear wheel gets speed * (100 - |angle|) / 100
    (exactly what picarx does) and the rear-axle centre moves at the average;
  * a dead time when starting from standstill, then a first-order lag;
  * bicycle steering with the default radius table (R = 30 cm at 30 deg);
  * the steering servo slews at 1 / servo_s_per_deg deg/s.
"""
import math

from obs_avoidance_profile_new.sim import SimCar

from . import calibration
from .geometry import Pose, footprint, polygon_distance


class ArenaSim(SimCar):
    TAU = 0.06              # s, first-order speed lag
    DEAD = 0.05             # s, dead time when starting from rest (matches the default lag_s)
    REST_CM_S = 0.5         # below this a coasting car is at rest

    def __init__(self, layout, car, start=None, tables=None, seed=0, invalid_rate=0.015,
                 spike_rate=0.01, strips=None):
        super().__init__(seed=seed, invalid_rate=invalid_rate, spike_rate=spike_rate)
        self.layout, self.car = layout, car
        t = tables or calibration.tables()
        self.l_eff = car.min_radius * math.tan(math.radians(30))
        self.slew = 1.0 / t["servo_s_per_deg"]
        self.dir_target = 0.0
        self.start_at = None
        p = start or layout.target("A")
        h = math.radians(p.h)
        self.x, self.y = p.x - car.rear_to_centre * math.cos(h), p.y - car.rear_to_centre * math.sin(h)
        self.phi = h
        self.obstacle = list(layout.o0)
        self.pull_t = None
        self.strips = strips or []          # [(x0, x1, y0, y1)] dark rectangles
        self.min_obstacle_clearance = float("inf")
        self.min_edge_clearance = float("inf")
        self.collided = False
        self.min_clearance = float("inf")
        self.box = self._box()

    # ---------------------------------------------------------- scenario
    def start_pull(self, delay=0.0):
        """The operator starts pulling the line `delay` s from now."""
        self.pull_t = self.t + delay

    def place_obstacle(self, where):
        self.obstacle = list(where)
        self.pull_t = None
        self.box = self._box()

    def pose(self):
        """Car-centre Pose."""
        o = self.car.rear_to_centre
        return Pose(self.x + o * math.cos(self.phi), self.y + o * math.sin(self.phi), math.degrees(self.phi))

    def _box(self):
        h = self.layout.obstacle_half
        cx, cy = self.obstacle
        return (cx - h, cx + h, cy - h, cy + h)

    # ---------------------------------------------------------- physics
    def _wheel_v(self, s):
        if s <= 0:
            return 0.0
        pwm = int(s / 2) + 50
        return 70.0 * (pwm - 35) / 65.0

    def _cmd_to_v(self, s):
        # picarx scales the inner wheel by the COMMANDED angle (dir_current_angle),
        # immediately, while the servo is still swinging
        a = min(30.0, abs(self.dir_target))
        return (self._wheel_v(abs(s)) + self._wheel_v(abs(s) * (100 - a) / 100.0)) / 2.0

    def forward(self, s):
        self._cmd(+1, s, "forward")

    def backward(self, s):
        self._cmd(-1, s, "backward")

    def _cmd(self, sign, s, name):
        if self.v == 0.0 and self.start_at is None:
            self.start_at = self.t + self.DEAD
        self.cmd = (sign, abs(s))
        self.motor_history.append((self.t, name, s))

    def stop(self):
        self.cmd = (0, 0)
        self.start_at = None
        self.motor_history.append((self.t, "stop", 0))

    def set_dir_servo_angle(self, a):
        self.dir_target = max(self.DIR_MIN, min(self.DIR_MAX, a))

    def _step(self, dt):
        sign, s = getattr(self, "cmd", (0, 0))
        if sign and (self.start_at is None or self.t >= self.start_at):
            self.v_target = sign * self._cmd_to_v(s)
        else:
            self.v_target = 0.0
        # servo slew
        d = self.dir_target - self.dir_angle
        self.dir_angle += max(-self.slew * dt, min(self.slew * dt, d))
        self.v += (self.v_target - self.v) * min(1.0, dt / self.TAU)
        if self.v_target == 0.0 and abs(self.v) < self.REST_CM_S:
            self.v = 0.0                    # friction: a coasting car does come to rest
        self.x += self.v * math.cos(self.phi) * dt
        self.y += self.v * math.sin(self.phi) * dt
        self.phi -= self.v / self.l_eff * math.tan(math.radians(self.dir_angle)) * dt
        # obstacle slides toward O1
        if self.pull_t is not None and self.t >= self.pull_t:
            tx, ty = self.layout.o1
            dx, dy = tx - self.obstacle[0], ty - self.obstacle[1]
            dist = math.hypot(dx, dy)
            step = self.layout.obstacle_slide_cm_s * dt
            if dist <= step:
                self.obstacle = [tx, ty]
                self.pull_t = None
            else:
                self.obstacle[0] += dx / dist * step
                self.obstacle[1] += dy / dist * step
            self.box = self._box()
        self.t += dt
        fp = footprint(self.pose(), self.car)
        x0, x1, y0, y1 = self.box
        c = polygon_distance(fp, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
        self.min_obstacle_clearance = min(self.min_obstacle_clearance, c)
        self.min_clearance = self.min_obstacle_clearance
        e = min(min(x, y, self.layout.width - x, self.layout.height - y) for x, y in fp)
        self.min_edge_clearance = min(self.min_edge_clearance, e)
        if c <= 0:
            self.collided = True

    # ---------------------------------------------------------- sensors
    def true_distance(self):
        p = self.pose()
        h = math.radians(p.h)
        sx = p.x + self.car.ultrasonic_offset * math.cos(h)
        sy = p.y + self.car.ultrasonic_offset * math.sin(h)
        best = None
        for k in range(-3, 4):
            a = h + math.radians(5 * k)
            d = self._ray_box(sx, sy, math.cos(a), math.sin(a))
            if d is not None and (best is None or d < best):
                best = d
        return best

    def get_grayscale_data(self):
        p = self.pose()
        h = math.radians(p.h)
        out = []
        for lat in (-2.0, 0.0, 2.0):           # three channels across the module
            x = p.x + self.car.grayscale_offset * math.cos(h) - lat * math.sin(h)
            y = p.y + self.car.grayscale_offset * math.sin(h) + lat * math.cos(h)
            dark = any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in self.strips)
            out.append(int((300 if dark else 1200) + self.rng.gauss(0, 30)))
        return out

    def heading_deg(self):
        return math.degrees(self.phi)
