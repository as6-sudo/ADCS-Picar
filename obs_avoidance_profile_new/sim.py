"""SimCar: a Picarx-compatible stand-in with simple physics, a box obstacle
and a noisy ultrasonic sensor. It is also its own clock (now/sleep), so a
whole profile runs in milliseconds of real time.

Model assumptions (rough, used only to check sequencing and safety margins):
  * motor PWM = 50 + speed/2 % (as in picarx); ground speed is
    V_MAX * (PWM - 35) / 65, so speed 100 ~ V_MAX cm/s;
  * velocity follows the command with a TAU first-order lag (coasting);
  * bicycle steering with ~25 cm turning radius at 30 deg;
  * sensor on the front bumper, +/-15 deg cone, ~1.5 % invalid readings,
    rare false short echoes while moving (as noted in time_traversal.core).
"""
import math
import random

# Car geometry (cm, relative to the rear axle centre)
REAR, FRONT, HALF_W = -4.0, 22.0, 9.5
MIN_RADIUS = 25.0
L_EFF = MIN_RADIUS * math.tan(math.radians(30))


class SimCar:
    DIR_MIN, DIR_MAX = -30, 30
    CAM_PAN_MIN, CAM_PAN_MAX = -90, 90
    CAM_TILT_MIN, CAM_TILT_MAX = -35, 65
    V_MAX = 70.0
    TAU = 0.08
    DT = 0.005
    READ_TIME = 0.012

    def __init__(self, obstacle_cm=120.0, seed=0, invalid_rate=0.015, spike_rate=0.01,
                 obstacle_half_width=15.0):
        self.rng = random.Random(seed)
        self.invalid_rate, self.spike_rate = invalid_rate, spike_rate
        self.t = 0.0
        self.x, self.y, self.phi = 0.0, 0.0, math.pi / 2    # facing +y (the obstacle)
        self.v = self.v_target = 0.0
        self.dir_angle = 0.0
        self.pan = self.tilt = 0.0
        y0 = FRONT + obstacle_cm
        self.box = (-obstacle_half_width, obstacle_half_width, y0, y0 + 20.0)
        self.pan_history = [(0.0, 0.0)]
        self.motor_history = []
        self.min_clearance = self._clearance()
        self.collided = False

    # ---------------------------------------------------------- clock
    def now(self):
        return self.t

    def sleep(self, s):
        while s > 0:
            dt = min(self.DT, s)
            self._step(dt)
            s -= dt

    def _step(self, dt):
        self.v += (self.v_target - self.v) * min(1.0, dt / self.TAU)
        self.x += self.v * math.cos(self.phi) * dt
        self.y += self.v * math.sin(self.phi) * dt
        self.phi -= self.v / L_EFF * math.tan(math.radians(self.dir_angle)) * dt
        self.t += dt
        c = self._clearance()
        self.min_clearance = min(self.min_clearance, c)
        if c <= 0:
            self.collided = True

    # ---------------------------------------------------------- picarx API
    def _speed_to_v(self, s):
        s = max(-100, min(100, s))
        if s == 0:
            return 0.0
        pwm = int(abs(s) / 2) + 50
        return self.V_MAX * (pwm - 35) / 65.0

    def forward(self, s):
        self.v_target = self._speed_to_v(s)
        self.motor_history.append((self.t, "forward", s))

    def backward(self, s):
        self.v_target = -self._speed_to_v(s)
        self.motor_history.append((self.t, "backward", s))

    def stop(self):
        self.v_target = 0.0
        self.motor_history.append((self.t, "stop", 0))

    def set_dir_servo_angle(self, a):
        self.dir_angle = max(self.DIR_MIN, min(self.DIR_MAX, a))

    def set_cam_pan_angle(self, a):
        self.pan = max(self.CAM_PAN_MIN, min(self.CAM_PAN_MAX, a))
        self.pan_history.append((self.t, self.pan))

    def set_cam_tilt_angle(self, a):
        self.tilt = max(self.CAM_TILT_MIN, min(self.CAM_TILT_MAX, a))

    def get_distance(self):
        self.sleep(self.READ_TIME)
        r = self.rng.random()
        if r < self.invalid_rate:
            return self.rng.choice((-1, -2))
        if abs(self.v) > 5 and r < self.invalid_rate + self.spike_rate:
            return round(self.rng.uniform(3, 10), 2)
        d = self.true_distance()
        if d is None or d > 343:
            return -1
        return round(d + self.rng.gauss(0, 0.5), 2)

    # ---------------------------------------------------------- geometry
    def heading_deg_cw(self):
        """Clockwise rotation from the start heading, in degrees."""
        return math.degrees(math.pi / 2 - self.phi)

    def _to_world(self, along, side):
        c, s = math.cos(self.phi), math.sin(self.phi)
        # side > 0 is the car's right
        return self.x + along * c + side * s, self.y + along * s - side * c

    def true_distance(self):
        sx, sy = self._to_world(FRONT, 0.0)
        best = None
        for k in range(-3, 4):
            a = self.phi + math.radians(5 * k)
            d = self._ray_box(sx, sy, math.cos(a), math.sin(a))
            if d is not None and (best is None or d < best):
                best = d
        return best

    def _ray_box(self, ox, oy, dx, dy):
        x0, x1, y0, y1 = self.box
        tmin, tmax = 0.0, float("inf")
        for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    return None
            else:
                t1, t2 = (lo - o) / d, (hi - o) / d
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
                if tmin > tmax:
                    return None
        return tmin

    def outline(self):
        pts = []
        n = 13
        for i in range(n + 1):
            a = REAR + (FRONT - REAR) * i / n
            pts += [self._to_world(a, -HALF_W), self._to_world(a, HALF_W)]
        for i in range(1, 10):
            w = -HALF_W + 2 * HALF_W * i / 10
            pts += [self._to_world(FRONT, w), self._to_world(REAR, w)]
        return pts

    def _clearance(self):
        x0, x1, y0, y1 = self.box
        best = float("inf")
        for px, py in self.outline():
            dx = max(x0 - px, 0.0, px - x1)
            dy = max(y0 - py, 0.0, py - y1)
            best = min(best, math.hypot(dx, dy))
        return best
