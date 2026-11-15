"""Car model and calibration.json (written by calibrate.py on the car).

Only the geometry part is used so far (plan.py). The speed, radius,
arc-speed, servo-swing and grayscale tables are added with the executor.
calibration.json is per car, so it is git-ignored.
"""
import json
import os
from dataclasses import dataclass, fields, replace

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")


@dataclass(frozen=True)
class CarModel:
    """Car geometry. Along-car values are measured from the REAR-AXLE CENTRE."""
    # Initial values from the brief (section 4); replace with measured ones.
    rear: float = -4.0              # rear bumper, cm behind the rear axle
    front: float = 22.0             # front bumper, cm ahead of the rear axle
    half_width: float = 9.5         # 19 cm wide
    rear_to_centre: float = 9.0     # geometric centre, cm ahead of the rear axle (brief 3, estimate)
    min_radius: float = 30.0        # rear-axle radius at full lock (30°); design assumption, brief 4
    # Distance needed to ramp speed up (or down) on a straight. Starting
    # guess ~8 cm (user decision); calibrate.py measures it.
    ramp_cm: float = 8.0
    # Ultrasonic sensor and grayscale module, cm ahead of the car CENTRE.
    ultrasonic_offset: float = 13.0     # front bumper (22 - 9); measure
    grayscale_offset: float = 10.0      # estimate (module sits under the front); measure


DEFAULT_CAR = CarModel()


def load(path=PATH):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save(data, path=PATH):
    with open(path, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)


def car_model(cal=None):
    """CarModel with any measured values from calibration.json's "car" section."""
    measured = (cal or {}).get("car", {})
    known = {f.name for f in fields(CarModel)}
    return replace(DEFAULT_CAR, **{k: float(v) for k, v in measured.items() if k in known})
