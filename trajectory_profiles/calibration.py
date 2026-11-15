"""Car model, calibration tables and calibration.json (written by calibrate.py).

calibration.json is per car, so it is git-ignored. Anything missing from it
falls back to the defaults below, VALUE BY VALUE: a value is used only if
the calibrate.py check that measures it passed (VALUE_CHECK). The defaults
are ESTIMATES (they match the simulator in sim.py). Calibration never blocks
a run: run.py only caps speed at 50 % while the speed or radius table is
missing (unless --no-cap), and warns about everything else.
"""
import json
import math
import os
from dataclasses import dataclass, replace

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")

# calibrate.py checks, in the order they should be run (README "Calibration order").
CHECKS = ("api", "ultrasonic", "motors", "footprint", "steering_trim", "speed_table",
          "radius_table", "servo_swing", "grayscale_threshold", "grayscale_offset")


@dataclass(frozen=True)
class CarModel:
    """Car geometry. Along-car values are measured from the REAR-AXLE CENTRE."""
    # Initial values from the brief (section 4); replace with measured ones.
    rear: float = -4.0              # rear bumper, cm behind the rear axle
    front: float = 22.0             # front bumper, cm ahead of the rear axle
    half_width: float = 9.5         # 19 cm wide
    rear_to_centre: float = 9.0     # geometric centre, cm ahead of the rear axle (brief 3, estimate)
    min_radius: float = 30.0        # rear-axle radius at full lock (30°); design assumption, brief 4
    # Distance used to ramp speed up (or down) on a straight. ~8 cm default
    # (user decision); MAIN and ALT have lead-ins of this length.
    ramp_cm: float = 8.0
    # Ultrasonic sensor and grayscale module, cm ahead of the car CENTRE.
    ultrasonic_offset: float = 13.0     # front bumper (22 - 9); measure
    grayscale_offset: float = 10.0      # estimate (module sits under the front); calibrate.py measures


DEFAULT_CAR = CarModel()

# Calibration tables. Defaults are estimates that match sim.py's car.
DEFAULT_TABLES = {
    # Straight ground speed per speed command: [command, cm/s, lag_s].
    # lag_s is the effective dead time of a start+stop: d = v * (t - lag).
    # calibrate.py fits v and lag from two timed runs per command.
    # Defaults: picarx PWM = 50 + s/2, ground speed ~ 70 * (PWM - 35) / 65 (sim.py).
    # Command 1 is the slowest the car can go: picarx never sends less than
    # 50 % PWM for a non-zero command, so there is a minimum speed.
    "speed_table": [[c, round(70.0 * (int(c / 2) + 15) / 65.0, 2), 0.05]
                    for c in (1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100)],
    # Rear-axle turning radius per steering angle: [deg, cm]. Defaults are the
    # bicycle model with R(30°) = 30 cm; calibrate.py drives full circles.
    "radius_table": [[a, round(30.0 * math.tan(math.radians(30)) / math.tan(math.radians(a)), 1)]
                     for a in (5, 10, 15, 20, 25, 30)],
    # Arc speed. picarx sends the inner rear wheel speed * (100 - |angle|) / 100,
    # so motion.py predicts the rear-axle speed on an arc as the mean of the two
    # wheels' straight-line speeds from speed_table, times this correction
    # factor per |steering angle| (wheel slip, tyre scrub). calibrate.py fits it
    # from timed full circles; default 1.0 = the model alone.
    "arc_speed_correction": [[0, 1.0], [30, 1.0]],
    # Speed response time constant (s): after a speed-command change the car's
    # position lags by (change in speed) x tau. Used to time ramps and the speed
    # changes between arcs and straights. Estimate (motor + drivetrain); if
    # repeat.py shows a speed-dependent along-track bias, tune this.
    "speed_tau_s": 0.06,
    "steer_trim_deg": 0.0,              # added to every steering command (steering_trim check)
    "servo_s_per_deg": 0.004,           # steering swing time; 0.24 s for -30 -> +30 (estimate)
    "servo_settle_s": 0.1,              # extra wait after a swing when stopped (LH)
    # After stop() the car coasts; wait this long before steering or reversing,
    # or the coast happens with the wrong steering. Estimate (3 x the sim lag).
    "stop_settle_s": 0.2,
    # Grayscale: a channel reading below its threshold is "dark" (tape).
    "grayscale_threshold": [700, 700, 700],
    # How far past the strip the car comes to rest after a tape stop, cm,
    # per cm/s of approach speed: [cm_s, cm]. Strips are placed this much early.
    "tape_overshoot": [[10.0, 0.5], [50.0, 2.0]],
}


def load(path=PATH):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save(data, path=PATH):
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)


def passed(data, check):
    return bool(data.get("checks", {}).get(check))


def all_passed(data):
    return all(passed(data, c) for c in CHECKS)


# Which calibrate.py check measures each value. A value is used only if its
# check PASSED (and the value is in the file); otherwise the default is used.
VALUE_CHECK = {
    "car.rear": "footprint", "car.front": "footprint", "car.half_width": "footprint",
    "car.rear_to_centre": "footprint", "car.ultrasonic_offset": "footprint",
    "steer_trim_deg": "steering_trim",
    "speed_table": "speed_table",
    "radius_table": "radius_table", "arc_speed_correction": "radius_table", "car.min_radius": "radius_table",
    "servo_s_per_deg": "servo_swing",
    "grayscale_threshold": "grayscale_threshold",
    "car.grayscale_offset": "grayscale_offset", "tape_overshoot": "grayscale_offset",
}
# Values no check measures: always the estimates in this file.
UNMEASURED = ("car.ramp_cm", "speed_tau_s", "servo_settle_s", "stop_settle_s")

# Without these, distance->time or radius->steering is a guess: cap speed at 50 %.
SPEED_CRITICAL = ("speed_table", "radius_table")
UNCALIBRATED_SPEED_SCALE = 0.5      # brief 6.6

# What else is affected when a check is missing or failed (warnings only).
EFFECT = {
    "api": "the installed picarx was not checked",
    "ultrasonic": "the ultrasonic was not checked: sensor stops and the hard stop are unverified",
    "motors": "motor directions were not checked",
    "footprint": "clearances use the brief's estimated car size",
    "steering_trim": "no steering trim: straights may drift",
    "servo_swing": "steering-swing timing is an estimate (HH/HL arcs)",
    "grayscale_threshold": "TAPE STOPS DISABLED: legs end on timed stops",
    "grayscale_offset": "tape strip positions use the estimated grayscale offset",
}


def _value(cal, key):
    if key.startswith("car."):
        return cal.get("car", {}).get(key[4:])
    return cal.get(key)


class Resolved:
    """calibration.json resolved value by value.

    measured: values whose check passed; defaults: [(value, reason)] for the
    rest; missing_checks: checks never run or failed."""

    def __init__(self, cal=None):
        cal = cal or {}
        self.checks = dict(cal.get("checks", {}))
        self.measured, self.defaults = [], []
        car_kw, tab = {}, dict(DEFAULT_TABLES)
        for key, check in VALUE_CHECK.items():
            v = _value(cal, key)
            state = self.checks.get(check)
            if state and v is not None:
                self.measured.append(key)
                if key.startswith("car."):
                    car_kw[key[4:]] = float(v)
                else:
                    tab[key] = v
            else:
                reason = ("check failed" if state is False else
                          "check not run" if check not in self.checks else "not in file")
                self.defaults.append((key, "%s (%s)" % (reason, check)))
        self.car = replace(DEFAULT_CAR, **car_kw)
        self.tables = tab
        self.missing_checks = [c for c in CHECKS if not self.checks.get(c)]

    def speed_cap(self):
        """0.5 if speed or turning calibration is missing/failed, else None."""
        bad = [c for c in SPEED_CRITICAL if c in self.missing_checks]
        return (UNCALIBRATED_SPEED_SCALE, bad) if bad else (None, [])

    @property
    def tape_stops(self):
        return "grayscale_threshold" not in self.missing_checks

    def warnings(self):
        out = []
        if self.defaults:
            out.append("Calibration: using DEFAULTS for %s" % ", ".join(k for k, _ in self.defaults))
            for k, why in self.defaults:
                out.append("    %-22s %s" % (k, why))
        for c in self.missing_checks:
            if c in EFFECT:
                out.append("  -> %s: %s" % (c, EFFECT[c]))
        out.append("Estimates (no check measures them): %s" % ", ".join(UNMEASURED))
        return out


def resolve(cal=None):
    return Resolved(cal)


def car_model(cal=None):
    """CarModel: measured values whose calibration check passed, defaults otherwise."""
    return Resolved(cal).car


def tables(cal=None):
    """Calibration tables: measured values whose check passed, defaults otherwise."""
    return Resolved(cal).tables


def startup(cal, *, sim=False, no_cap=False, speed_scale=None, out=print):
    """Resolve calibration for a run. Never refuses.

    Returns (Resolved, speed scale, tape_stops). Prints which values are
    defaults, what that affects, and whether speed is capped."""
    res = Resolved(cal)
    scale = 1.0 if speed_scale is None else speed_scale
    for line in res.warnings():
        out(line)
    if sim:
        # the simulated car's "truth" matches the defaults: no cap, tape stops on
        return res, scale, True
    cap, why = res.speed_cap()
    if no_cap:
        out("WARNING: --no-cap: speed is NOT capped%s"
            % (" although %s not calibrated" % " and ".join(why) if why else ""))
    elif cap is not None and scale > cap:
        out("Speed capped at %.0f%% because %s not calibrated (--no-cap removes the cap)."
            % (cap * 100, " and ".join(why)))
        scale = cap
    return res, scale, res.tape_stops
