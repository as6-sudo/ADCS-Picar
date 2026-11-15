"""Car model, calibration tables and calibration.json (written by calibrate.py).

calibration.json is per car, so it is git-ignored. Anything missing from it
falls back to the defaults below. The defaults are ESTIMATES (they match the
simulator in sim.py), good enough for --sim and for a first look at 50 %
speed, never for filming. run.py caps speed at 50 % until every check in
CHECKS has passed.
"""
import json
import math
import os
from dataclasses import dataclass, fields, replace

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


def car_model(cal=None):
    """CarModel with any measured values from calibration.json's "car" section."""
    measured = (cal or {}).get("car", {})
    known = {f.name for f in fields(CarModel)}
    return replace(DEFAULT_CAR, **{k: float(v) for k, v in measured.items() if k in known})


def tables(cal=None):
    """Calibration tables: measured values where present, defaults otherwise."""
    t = dict(DEFAULT_TABLES)
    t.update({k: v for k, v in (cal or {}).items() if k in DEFAULT_TABLES})
    return t
