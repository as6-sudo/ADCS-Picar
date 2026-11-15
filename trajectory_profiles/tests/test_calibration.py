"""Calibration never blocks a run (laptop, no hardware):

    python -m unittest discover -s trajectory_profiles/tests -t .

Scenarios: no calibration file, partial calibration, full calibration, --no-cap.
"""
import csv
import glob
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from trajectory_profiles import calibration, run  # noqa: E402
from trajectory_profiles.calibration import DEFAULT_CAR, DEFAULT_TABLES  # noqa: E402

FULL = {
    "checks": {c: True for c in calibration.CHECKS},
    "car": {"rear": -4.5, "front": 22.5, "half_width": 9.6, "rear_to_centre": 9.0, "ultrasonic_offset": 13.4,
            "min_radius": 29.5, "grayscale_offset": 9.0},
    "steer_trim_deg": -0.8,
    "speed_table": [[1, 15.0, 0.06], [30, 31.0, 0.06], [60, 47.0, 0.05], [90, 62.0, 0.05]],
    "radius_table": [[8, 120.0], [12, 82.0], [18, 53.0], [23, 41.0], [27, 34.0], [30, 29.5]],
    "arc_speed_correction": [[0, 1.0], [30, 0.97]],
    "servo_s_per_deg": 0.0035,
    "grayscale_threshold": [650, 640, 660],
    "tape_overshoot": [[17.0, 0.7], [50.0, 1.9]],
}


def partial():
    """Speed table passed, radius table failed, grayscale never run, footprint passed."""
    cal = json.loads(json.dumps(FULL))
    cal["checks"] = {"api": True, "ultrasonic": True, "motors": True, "footprint": True,
                     "speed_table": True, "radius_table": False}
    del cal["grayscale_threshold"]
    return cal


def quiet(fn, *a, **kw):
    out = io.StringIO()
    with redirect_stdout(out):
        r = fn(*a, **kw)
    return r, out.getvalue()


class Resolve(unittest.TestCase):
    def test_no_file(self):
        res = calibration.resolve(calibration.load(os.path.join(tempfile.gettempdir(), "does-not-exist.json")))
        self.assertEqual(res.measured, [])
        self.assertEqual(res.car, DEFAULT_CAR)
        self.assertEqual(res.tables["speed_table"], DEFAULT_TABLES["speed_table"])
        self.assertEqual(res.speed_cap(), (0.5, ["speed_table", "radius_table"]))
        self.assertFalse(res.tape_stops)
        self.assertIn("speed_table", " ".join(res.warnings()))

    def test_partial_uses_only_values_whose_check_passed(self):
        res = calibration.resolve(partial())
        self.assertEqual(res.tables["speed_table"], FULL["speed_table"])          # passed
        self.assertEqual(res.car.half_width, 9.6)                                  # footprint passed
        self.assertEqual(res.tables["radius_table"], DEFAULT_TABLES["radius_table"])   # failed -> default
        self.assertEqual(res.car.min_radius, DEFAULT_CAR.min_radius)
        self.assertEqual(res.tables["steer_trim_deg"], 0.0)                       # check never run
        reasons = dict(res.defaults)
        self.assertEqual(reasons["radius_table"], "check failed (radius_table)")
        self.assertEqual(reasons["steer_trim_deg"], "check not run (steering_trim)")
        self.assertEqual(res.speed_cap(), (0.5, ["radius_table"]))
        self.assertFalse(res.tape_stops)

    def test_full(self):
        res = calibration.resolve(FULL)
        self.assertEqual(res.defaults, [])
        self.assertEqual(res.speed_cap(), (None, []))
        self.assertTrue(res.tape_stops)
        self.assertEqual(res.car.min_radius, 29.5)
        self.assertEqual(res.tables["servo_s_per_deg"], 0.0035)

    def test_only_speed_and_radius_cap_the_speed(self):
        cal = json.loads(json.dumps(FULL))
        for c in ("grayscale_threshold", "grayscale_offset", "servo_swing", "footprint", "steering_trim"):
            cal["checks"][c] = False
        res = calibration.resolve(cal)
        self.assertEqual(res.speed_cap(), (None, []))
        self.assertFalse(res.tape_stops)              # only the affected feature goes

    def test_startup_scale(self):
        out = []
        _, s, _ = calibration.startup({}, speed_scale=None, out=out.append)
        self.assertEqual(s, 0.5)
        _, s, _ = calibration.startup({}, no_cap=True, out=out.append)
        self.assertEqual(s, 1.0)
        self.assertTrue(any("--no-cap" in line for line in out))
        _, s, _ = calibration.startup(FULL, out=out.append)
        self.assertEqual(s, 1.0)
        _, s, _ = calibration.startup({}, sim=True, out=out.append)
        self.assertEqual(s, 1.0)                      # the simulator is never capped


class RunAlwaysRuns(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.logs = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
        self.before = set(glob.glob(os.path.join(self.logs, "*.csv")))

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)
        for f in set(glob.glob(os.path.join(self.logs, "*.csv"))) - self.before:
            os.remove(f)

    def cal_file(self, data):
        p = os.path.join(self.dir, "calibration.json")
        if data is not None:
            with open(p, "w") as f:
                json.dump(data, f)
        return p

    def test_dry_run_with_no_partial_and_full_calibration(self):
        for name, data, capped in (("none", None, True), ("partial", partial(), True), ("full", FULL, False)):
            with self.subTest(name):
                code, out = quiet(run.main, ["HH", "--dry-run", "--calibration", self.cal_file(data)])
                self.assertEqual(code, 0)
                self.assertEqual("Speed capped at 50%" in out, capped)
                if name != "full":
                    self.assertIn("using DEFAULTS for", out)

    def test_no_cap_flag(self):
        code, out = quiet(run.main, ["HH", "--dry-run", "--no-cap", "--calibration", self.cal_file(None)])
        self.assertEqual(code, 0)
        self.assertNotIn("Speed capped", out)
        self.assertIn("WARNING: --no-cap", out)

    def test_csv_lists_measured_and_default_values(self):
        code, out = quiet(run.main, ["HL", "--sim", "--leg", "1", "--calibration", self.cal_file(partial())])
        self.assertEqual(code, 0)
        new = sorted(set(glob.glob(os.path.join(self.logs, "*.csv"))) - self.before)
        self.assertEqual(len(new), 1)
        with open(new[0], newline="") as f:
            rows = [r for r in csv.DictReader(f) if r["kind"] == "calibration"]
        text = " ".join(r["detail"] for r in rows)
        self.assertIn("measured:", text)
        self.assertIn("speed_table", text.split("defaults:")[0])               # measured
        self.assertIn("radius_table[check failed (radius_table)]", text)       # default, with the reason


if __name__ == "__main__":
    unittest.main()
