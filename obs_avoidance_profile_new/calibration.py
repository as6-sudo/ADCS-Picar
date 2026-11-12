"""Load/save calibration.json (written by calibrate.py on the car)."""
import json
import os

from .params import DEFAULT_MAX_SAFE_SPEED

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration.json")
CHECKS = ("api", "ultrasonic", "motors", "camera_pan", "steering", "stop_distance")


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


def max_safe_speed(data):
    return int(data.get("max_safe_speed", DEFAULT_MAX_SAFE_SPEED))
