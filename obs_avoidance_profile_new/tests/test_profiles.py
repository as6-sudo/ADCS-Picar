"""Simulation and compatibility tests. Standard library only:

    python -m unittest discover -s obs_avoidance_profile_new/tests -t .

On the Raspberry Pi the API test also checks the real installed picarx.
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from obs_avoidance_profile_new import calibration  # noqa: E402
from obs_avoidance_profile_new.params import A, B, C, D, DETECT_CM, HARD_MIN_CM, PROFILES  # noqa: E402
from obs_avoidance_profile_new.robot import EStop, Robot, validate_distance  # noqa: E402
from obs_avoidance_profile_new.runner import run_profile  # noqa: E402
from obs_avoidance_profile_new.sim import SimCar  # noqa: E402

SEEDS = range(5)
MAX_SPEED = 70


def simulate(P, seed=0, obstacle_cm=120.0, estop_at=None, car=None):
    car = car or SimCar(obstacle_cm=obstacle_cm, seed=seed)
    estop = EStop()
    if estop_at is not None:
        sleep = car.sleep

        def sleep_and_trip(s):
            sleep(s)
            if car.t >= estop_at:
                estop.trigger("test")
        car.sleep = sleep_and_trip
    r = Robot(car, clock=car, estop=estop, log=lambda *a: None)
    ok = run_profile(r, P, MAX_SPEED, log=lambda *a: None)
    return ok, car, r


def states(r):
    return [d["name"] for _, k, d in r.trace if k == "state"]


def pan_sweeps(car):
    """Count left/right extremes reached by the camera head."""
    n, side = 0, 0
    for _, a in car.pan_history:
        s = (a >= 20) - (a <= -20)
        if s and s != side:
            n, side = n + 1, s
        elif a == 0:
            side = 0
    return n


def max_pan_rate(car):
    h = car.pan_history
    return max((abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(h, h[1:]) if b[0] > a[0]), default=0)


def approach_speed(r):
    return next(d["speed"] for _, k, d in r.trace if k == "drive")


class SafetyAllProfiles(unittest.TestCase):
    def test_no_collision_and_ends_stopped_and_neutral(self):
        for P in PROFILES.values():
            for seed in SEEDS:
                for dist in (60.0, 120.0, 200.0):
                    with self.subTest(profile=P.key, seed=seed, dist=dist):
                        ok, car, r = simulate(P, seed, dist)
                        self.assertTrue(ok)
                        self.assertFalse(car.collided)
                        self.assertGreater(car.min_clearance, 8.0)
                        self.assertEqual(car.motor_history[-1][1], "stop")
                        self.assertEqual((car.pan, car.dir_angle, car.tilt), (0, 0, 0))

    def test_stops_before_threshold_region(self):
        for P in (A, B, C):
            ok, car, r = simulate(P)
            det = [d["dist"] for _, k, d in r.trace if k == "detect"][0]
            self.assertLess(det, DETECT_CM)
            self.assertGreater(det, HARD_MIN_CM)

    def test_estop_halts_motion(self):
        for P in PROFILES.values():
            ok, car, r = simulate(P, estop_at=1.0)
            self.assertFalse(ok)
            self.assertEqual(car.motor_history[-1][1], "stop")
            t_stop = car.t
            car.sleep(1.0)                      # coasts to rest, no more commands
            self.assertLess(abs(car.v), 0.5)
            self.assertLess(t_stop, 1.5)

    def test_error_inside_profile_still_stops_motors(self):
        car = SimCar()
        calls = {"n": 0}
        real = car.get_distance

        def broken():
            calls["n"] += 1
            if calls["n"] > 20:
                raise RuntimeError("sensor unplugged")
            return real()
        car.get_distance = broken
        with self.assertRaises(RuntimeError):
            simulate(A, car=car)
        self.assertEqual(car.motor_history[-1][1], "stop")

    def test_no_obstacle_aborts_and_stops(self):
        car = SimCar(obstacle_cm=2000.0)
        ok, car, r = simulate(B, car=car)
        self.assertFalse(ok)
        self.assertEqual(car.motor_history[-1][1], "stop")


class SensorValidation(unittest.TestCase):
    def test_invalid_values_rejected(self):
        for raw in (-1, -2, 0, 1.0, 500, None, "x", float("nan")):
            self.assertIsNone(validate_distance(raw))
        self.assertEqual(validate_distance(42.5), 42.5)

    def test_single_false_echo_does_not_stop(self):
        car = SimCar(invalid_rate=0, spike_rate=0)
        seq = iter([100, 5, 100, 4, 100, -1, -2] * 3)
        real = car.get_distance
        car.get_distance = lambda: next(seq, None) or real()
        r = Robot(car, clock=car, log=lambda *a: None)
        d = r.approach(40, DETECT_CM, 8)
        self.assertLess(d, DETECT_CM)
        self.assertGreater(car.y, 80)          # did not stop at the fake echoes


class ProfileA(unittest.TestCase):
    def test_sequence(self):
        ok, car, r = simulate(A)
        names = states(r)
        self.assertEqual([n for n in names if n.startswith("SCAN")], ["SCAN_1", "SCAN_2", "SCAN_3"])
        self.assertEqual([n for n in names if n.startswith("REVERSE")], ["REVERSE_1", "REVERSE_2", "REVERSE_3"])
        self.assertEqual(names[-2:], ["EXIT_TURN_RIGHT", "EXIT_STRAIGHT"])
        self.assertEqual(pan_sweeps(car), 3 * 3 * 2)       # 3 scans x 3 left-right sweeps

    def test_exits_right(self):
        for seed in SEEDS:
            ok, car, r = simulate(A, seed)
            self.assertTrue(60 <= car.heading_deg_cw() <= 120)
            self.assertGreater(car.x, 40)


class ProfileB(unittest.TestCase):
    def test_sequence_and_uturn_right(self):
        for seed in SEEDS:
            ok, car, r = simulate(B, seed)
            self.assertEqual(sum(n.startswith("SCAN") for n in states(r)), 1)
            self.assertNotIn("REVERSE_1", states(r))
            self.assertEqual(pan_sweeps(car), 2)
            self.assertTrue(120 <= car.heading_deg_cw() <= 200)
            self.assertGreater(car.x, 0)

    def test_fastest_and_quickest(self):
        res = {P.key: simulate(P) for P in (A, B, C)}
        speeds = {k: approach_speed(v[2]) for k, v in res.items()}
        self.assertEqual(max(speeds, key=speeds.get), "B")
        self.assertLess(res["B"][1].t, res["A"][1].t)
        self.assertLess(res["B"][1].t, res["C"][1].t)


class ProfileC(unittest.TestCase):
    def test_sequence(self):
        ok, car, r = simulate(C)
        names = states(r)
        i1, rv, i2, u = (names.index(n) for n in ("SCAN_1", "REVERSE_1", "SCAN_2", "UTURN_REVERSE_LEFT_WHEELS"))
        self.assertTrue(i1 < rv < i2 < u)
        self.assertEqual(pan_sweeps(car), 4)               # 2 scans x 1 sweep

    def test_slower_than_b_and_exits_right(self):
        okc, carc, rc = simulate(C)
        okb, carb, rb = simulate(B)
        self.assertLess(approach_speed(rc), approach_speed(rb))
        self.assertLess(C.turn_speed, B.turn_speed)
        self.assertLess(max_pan_rate(carc), max_pan_rate(carb))
        for seed in SEEDS:
            ok, car, r = simulate(C, seed)
            self.assertTrue(120 <= car.heading_deg_cw() <= 200)
            self.assertGreater(car.x, 0)


class PanSpeedOrdering(unittest.TestCase):
    def test_a_fastest_then_b_then_c(self):
        rates = {P.key: max_pan_rate(simulate(P)[1]) for P in (A, B, C)}
        self.assertGreater(rates["A"], rates["B"])
        self.assertGreater(rates["B"], rates["C"])


class ProfileD(unittest.TestCase):
    def test_persistent_no_scan_no_exit(self):
        for seed in SEEDS:
            ok, car, r = simulate(D, seed)
            self.assertEqual(pan_sweeps(car), 0)
            self.assertTrue(all(a == 0 for _, a in car.pan_history))
            self.assertAlmostEqual(car.heading_deg_cw(), 0, delta=1)
            self.assertLess(car.true_distance(), 40)            # stays at the obstacle
            self.assertEqual(sum(n.startswith("ATTEMPT") for n in states(r)), D.attempts)
            self.assertFalse(car.collided)
            self.assertGreater(car.min_clearance, HARD_MIN_CM - 4)


class Compatibility(unittest.TestCase):
    METHODS = ("forward", "backward", "stop", "set_dir_servo_angle",
               "set_cam_pan_angle", "set_cam_tilt_angle", "get_distance")

    def test_simcar_matches_picarx_surface(self):
        for m in self.METHODS:
            self.assertTrue(callable(getattr(SimCar, m)))
        self.assertEqual((SimCar.DIR_MIN, SimCar.DIR_MAX), (-30, 30))

    def test_real_picarx_if_installed(self):
        try:
            from picarx import Picarx
        except Exception:
            self.skipTest("picarx not installed (not on the car)")
        for m in self.METHODS:
            self.assertTrue(callable(getattr(Picarx, m)), m)
        self.assertEqual((Picarx.DIR_MIN, Picarx.DIR_MAX), (-30, 30))
        self.assertGreaterEqual(Picarx.CAM_PAN_MAX, 60)

    def test_time_traversal_still_runs(self):
        """The reference package is untouched and still works with a Picarx-like car."""
        from time_traversal.core import run_profile as tt_run
        from time_traversal.profiles import PROFILES as TT
        for cls in TT.values():
            car = SimCar()
            tt_run(cls(seed=1), car, duration=0.3, log=False)
            self.assertEqual(car.motor_history[-1][1], "stop")

    def test_uncalibrated_defaults(self):
        self.assertFalse(calibration.all_passed({}))
        self.assertEqual(calibration.max_safe_speed({}), 70)
        self.assertTrue(calibration.all_passed({"checks": {c: True for c in calibration.CHECKS}}))


if __name__ == "__main__":
    unittest.main()
