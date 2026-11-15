"""Distance -> time and radius -> steering conversions from the calibration
tables (brief 6.1). No magic constants: every number comes from
calibration.tables().
"""
from .geometry import LEFT, RIGHT, STRAIGHT

MAX_STEER = 30.0


def interp(table, x):
    """Piecewise-linear interpolation in a [[x, y], ...] table, clamped at the ends."""
    pts = sorted((float(a), float(b)) for a, b, *_ in table)
    if x <= pts[0][0]:
        return pts[0][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return pts[-1][1]


class Motion:
    def __init__(self, tables):
        self.t = tables

    # ---------------------------------------------------------- speed
    @staticmethod
    def pwm(cmd):
        """picarx: a non-zero command s is sent as PWM = int(s / 2) + 50 (%)."""
        return 0 if cmd <= 0 else int(cmd / 2) + 50

    def straight_speed(self, cmd):
        """Straight ground speed for a command, interpolated between the measured
        commands on picarx's integer PWM scale (so the truncation is modelled)."""
        if cmd <= 0:
            return 0.0
        return interp([[self.pwm(r[0]), r[1]] for r in self.t["speed_table"]], self.pwm(cmd))

    def lag(self, cmd):
        return interp([[r[0], r[2]] for r in self.t["speed_table"]], cmd)

    def ground_speed(self, cmd, steer):
        """Rear-axle ground speed (cm/s) for a speed command at a steering angle.

        picarx scales the inner rear wheel's command by (100 - |angle|) / 100;
        the rear-axle centre moves at the mean of the two wheel speeds."""
        a = min(MAX_STEER, abs(steer))
        inner = self.straight_speed(cmd * (100.0 - a) / 100.0) if cmd > 0 else 0.0
        corr = interp(self.t["arc_speed_correction"], a)
        return corr * (self.straight_speed(cmd) + inner) / 2.0

    def command_for(self, cm_s, steer=0.0):
        """Smallest-error speed command (1..100) for `cm_s` of ground speed at `steer`."""
        return min(range(1, 101), key=lambda c: (abs(self.ground_speed(c, steer) - cm_s), c))

    def max_speed(self, steer=0.0):
        return self.ground_speed(100, steer)

    # ---------------------------------------------------------- steering
    def steer_for(self, side, radius):
        """Servo angle for a rear-axle radius. +angle steers RIGHT (picarx).

        Interpolated in curvature (1/R), which is close to linear in the
        steering angle, between the measured points of the radius table."""
        if side == STRAIGHT:
            a = 0.0
        else:
            table = sorted((1.0 / r, deg) for deg, r in self.t["radius_table"])
            a = interp([[0.0, 0.0]] + [list(p) for p in table], 1.0 / radius)
            a = a if side == RIGHT else -a
        return max(-MAX_STEER, min(MAX_STEER, a + self.t["steer_trim_deg"]))

    def min_radius(self):
        return min(r for deg, r in self.t["radius_table"] if abs(deg) <= MAX_STEER)

    def swing_time(self, a, b):
        return abs(b - a) * self.t["servo_s_per_deg"]

    # ---------------------------------------------------------- tape
    def tape_overshoot(self, cm_s):
        return interp(self.t["tape_overshoot"], cm_s)


def side_of(angle):
    return STRAIGHT if abs(angle) < 1e-9 else (RIGHT if angle > 0 else LEFT)


def check_radius_consistency(motion, car):
    """Problems if the calibrated table can't reach the car model's min radius."""
    r = motion.min_radius()
    if r > car.min_radius + 0.5:
        return ["calibrated minimum radius %.1f cm > car model %.1f cm: rerun plan.py --solve" % (r, car.min_radius)]
    return []

