#!/usr/bin/env python3
"""Interactive calibration ON THE CAR. Results go to calibration.json.

    python3 trajectory_profiles/calibrate.py                  # every check, in order
    python3 trajectory_profiles/calibrate.py --only speed_table

Checks (README "Calibration order" explains each one):
  api                  installed picarx has every method these profiles use
  ultrasonic           30 readings at 30 cm (same check as obs_avoidance_profile_new)
  motors               short forward then reverse blip
  footprint            enter the measured car geometry
  steering_trim        drive 150 cm straight, enter the sideways drift; repeat until < 2 cm
  speed_table          two timed runs per speed command, enter the distances -> cm/s + lag
  radius_table         one arc per steering angle, enter where the rear axle ended
                       -> radius + arc-speed correction (an arc, not a full circle,
                       so the big radii fit in the arena)
  servo_swing          film the wheels in slow motion, enter the -30 -> +30 time
  grayscale_threshold  floor vs tape readings -> per-channel threshold
  grayscale_offset     where the module is, and how far the car coasts past a strip

Tools: a tape measure, a marker, a strip of the tape you use for the stops,
and a phone that films slow motion. Emergency stop: Ctrl+C.
"""
import argparse
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration  # noqa: E402
from trajectory_profiles import geometry as g  # noqa: E402
from trajectory_profiles.executor import Executor, TapeSpec, ThreadedRanger  # noqa: E402
from trajectory_profiles.motion import Motion  # noqa: E402

REQUIRED_METHODS = ("forward", "backward", "stop", "set_dir_servo_angle", "set_cam_pan_angle",
                    "set_cam_tilt_angle", "get_distance", "get_grayscale_data")
TEST_CMD = 30                                   # gentle speed command for the checks
SPEED_CMDS = (1, 15, 30, 45, 60, 75, 90)        # covers 16 .. ~65 cm/s
RADIUS_ANGLES = (8, 12, 18, 23, 27, 30)         # spans every radius the paths use (R30..R120)


def ask(q):
    return input(q + " [y/n] ").strip().lower().startswith("y")


def ask_float(q, default=None):
    while True:
        s = input("%s%s: " % (q, " [%s]" % default if default is not None else "")).strip()
        if not s and default is not None:
            return float(default)
        try:
            return float(s)
        except ValueError:
            print("  a number please")


# ------------------------------------------------------------------ checks
def check_api(ctx):
    car = ctx["car"]
    missing = [m for m in REQUIRED_METHODS if not callable(getattr(car, m, None))]
    print("  missing methods:", missing or "none", "| DIR_MAX:", getattr(car, "DIR_MAX", None))
    return not missing and getattr(car, "DIR_MAX", None) == 30


def check_ultrasonic(ctx):
    from obs_avoidance_profile_new.calibrate import check_ultrasonic as obs_check
    return obs_check(ctx["car"], None)


def check_motors(ctx):
    ex = ctx["ex"]
    input("  Wheels on the floor, 50 cm clear front and back. Press Enter.")
    ex.run([g.seg("F 0 10")], leg="cal", speed=ctx["m"].straight_speed(TEST_CMD), rolling=False, guard=False)
    time.sleep(0.5)
    ex.run([g.seg("B 0 10")], leg="cal", speed=ctx["m"].straight_speed(TEST_CMD), rolling=False)
    return ask("  Did it go FORWARD about 10 cm first, then BACKWARD?")


def check_footprint(ctx):
    cal = ctx["cal"]
    c = calibration.car_model(cal)
    print("  Measure along the car from the REAR-AXLE CENTRE (cm; + = forward).")
    rear = -abs(ask_float("  rear bumper, cm behind the rear axle", -c.rear))
    front = ask_float("  front bumper, cm ahead of the rear axle", c.front)
    width = ask_float("  overall width, cm (widest point: wheels or body)", 2 * c.half_width)
    us = ask_float("  ultrasonic sensor face, cm ahead of the rear axle", c.ultrasonic_offset + c.rear_to_centre)
    centre = (front + rear) / 2.0
    cal.setdefault("car", {}).update(rear=rear, front=front, half_width=width / 2, rear_to_centre=centre,
                                     ultrasonic_offset=us - centre)
    print("  car centre is %.1f cm ahead of the rear axle" % centre)
    print("  NOTE: run plan.py on the laptop with this calibration.json; it must still pass.")
    return True


def check_steering_trim(ctx):
    cal, m, ex = ctx["cal"], ctx["m"], ctx["ex"]
    length = 150.0
    while True:
        m.t["steer_trim_deg"] = cal.get("steer_trim_deg", 0.0)
        input("  Put the car at the start of a 150 cm straight line (mark the rear-axle centre). Enter.")
        ex.run([g.Segment("F", "0", 0, length)], leg="cal", speed=m.straight_speed(TEST_CMD), rolling=False,
               guard=False)
        drift = ask_float("  Sideways drift of the rear-axle centre at the end, cm (+ = right, - = left)")
        if abs(drift) <= 2.0:
            print("  trim %.2f deg is good" % m.t["steer_trim_deg"])
            return True
        # constant curvature k gives drift = k L^2 / 2; steering for k on the bicycle model
        l_eff = calibration.car_model(cal).min_radius * math.tan(math.radians(30))
        k = 2 * drift / length ** 2
        delta = math.degrees(math.atan(l_eff * k))
        cal["steer_trim_deg"] = round(cal.get("steer_trim_deg", 0.0) - delta, 2)   # drifted right -> steer left
        print("  new trim %.2f deg; drive again" % cal["steer_trim_deg"])
        calibration.save(cal)
        if not ask("  Drive again?"):
            return False


def check_speed_table(ctx):
    cal, m, car = ctx["cal"], ctx["m"], ctx["car"]
    rows = []
    print("  For each command the car drives straight twice; measure how far the rear axle moved.")
    for cmd in SPEED_CMDS:
        guess = m.straight_speed(cmd)
        t2 = min(2.0, 100.0 / guess)                 # stay inside ~1 m
        t1 = t2 / 2
        ds = []
        for t in (t1, t2):
            input("  cmd %d, %.2f s: place the car (mark the rear axle), 1.2 m clear ahead. Enter." % (cmd, t))
            car.set_dir_servo_angle(m.t["steer_trim_deg"])
            time.sleep(0.4)
            car.forward(cmd)
            time.sleep(t)
            car.stop()
            time.sleep(0.5)
            ds.append(ask_float("  distance travelled, cm"))
        v = (ds[1] - ds[0]) / (t2 - t1)
        if v <= 0:
            print("  second run was not longer than the first; skipping cmd %d" % cmd)
            continue
        lag = t1 - ds[0] / v
        print("  cmd %d: %.1f cm/s, lag %.3f s (start-up distance ~%.1f cm)" % (cmd, v, lag, v * lag))
        rows.append([cmd, round(v, 2), round(max(0.0, lag), 3)])
    if len(rows) < 3:
        return False
    cal["speed_table"] = rows
    startup = max(r[1] * r[2] for r in rows)
    ramp = calibration.car_model(cal).ramp_cm
    if startup > ramp:
        print("  WARNING: start-up distance %.1f cm > ramp_cm %.1f: set car.ramp_cm and run plan.py --solve"
              % (startup, ramp))
    return True


def check_radius_table(ctx):
    cal, m, ex = ctx["cal"], ctx["m"], ctx["ex"]
    print("  Floor frame: x = the car's starting heading, y = to its LEFT, origin = the rear-axle")
    print("  centre at the start. After each arc, measure where the rear-axle centre ended up.")
    radius_rows, corr_rows = [], []
    for a in RADIUS_ANGLES:
        input("  Steering %d deg RIGHT: place the car (mark the rear axle), 1.5 m clear. Enter." % a)
        ex.steer_stopped(a)
        car = ctx["car"]
        t = 1.2
        car.forward(TEST_CMD)
        time.sleep(t)
        car.stop()
        time.sleep(0.5)
        car.set_dir_servo_angle(0)
        dx = ask_float("  x (forward), cm")
        dy = -abs(ask_float("  y (to the RIGHT, as a positive number), cm"))
        if abs(dy) < 1:
            print("  too straight to measure; skipping")
            continue
        r = (dx * dx + dy * dy) / (2 * abs(dy))
        theta = 2 * math.atan2(abs(dy), dx)
        arc_v = r * theta / (t - m.lag(TEST_CMD))
        # the wheel model alone (correction 1.0): mean of outer and slowed inner wheel
        model_v = (m.straight_speed(TEST_CMD) + m.straight_speed(TEST_CMD * (100 - a) / 100.0)) / 2
        print("  %d deg: R = %.1f cm, %.0f deg of turn, arc speed %.1f cm/s (model %.1f)"
              % (a, r, math.degrees(theta), arc_v, model_v))
        radius_rows.append([a, round(r, 1)])
        corr_rows.append([a, round(arc_v / model_v, 3)])
    if len(radius_rows) < 3:
        return False
    cal["radius_table"] = radius_rows
    cal["arc_speed_correction"] = [[0, 1.0]] + corr_rows
    rmin = min(r for _, r in radius_rows)
    cal.setdefault("car", {})["min_radius"] = rmin
    if rmin > 30.5:
        print("  WARNING: minimum radius %.1f cm > the 30 cm the paths assume: run plan.py --solve" % rmin)
    return True


def check_servo_swing(ctx):
    cal, ex = ctx["cal"], ctx["ex"]
    input("  Film the front wheels in slow motion. The steering swings -30 -> +30 five times. Enter.")
    for _ in range(5):
        ex.car.set_dir_servo_angle(-30)
        time.sleep(1.0)
        ex.car.set_dir_servo_angle(30)
        time.sleep(1.0)
    ex.car.set_dir_servo_angle(0)
    s = ask_float("  measured time for one -30 -> +30 swing, s (Enter keeps the estimate)",
                  60 * calibration.tables(cal)["servo_s_per_deg"])
    cal["servo_s_per_deg"] = round(s / 60.0, 5)
    return True


def _gray(car, n=30):
    vals = []
    for _ in range(n):
        vals.append(car.get_grayscale_data())
        time.sleep(0.02)
    return [statistics.median(v[i] for v in vals) for i in range(3)]


def check_grayscale_threshold(ctx):
    cal, car = ctx["cal"], ctx["car"]
    input("  Car on the bare floor of the arena. Enter.")
    floor = _gray(car)
    input("  Now with a strip of the stop tape under all three sensors. Enter.")
    tape = _gray(car)
    print("  floor", floor, "tape", tape)
    gaps = [f - t for f, t in zip(floor, tape)]
    if min(gaps) < 200:
        print("  floor and tape are too similar (gap %s); use darker tape" % min(gaps))
        return False
    cal["grayscale_threshold"] = [round((f + t) / 2) for f, t in zip(floor, tape)]
    print("  thresholds", cal["grayscale_threshold"])
    return True


def check_grayscale_offset(ctx):
    cal, car, ex = ctx["cal"], ctx["car"], ctx["ex"]
    cm = calibration.car_model(cal)
    thr = calibration.tables(cal)["grayscale_threshold"]
    print("  Static: slide a strip under the car from the front until this says DARK.")
    while True:
        dark = any(v < t for v, t in zip(car.get_grayscale_data(), thr))
        print("  ", "DARK " if dark else "light", end="\r")
        if dark:
            break
        time.sleep(0.05)
    d = ask_float("\n  Distance from the strip's centre line to the rear-axle centre, cm")
    offset = d - cm.rear_to_centre
    cal.setdefault("car", {})["grayscale_offset"] = round(offset, 1)
    print("  grayscale module is %.1f cm ahead of the car centre" % offset)
    rows = []
    for v in (17.0, 35.0, 50.0):         # the arrival speeds of HL, LH and HH (approx.)
        if not ask("  Dynamic test at %.0f cm/s (drives at a strip ~40 cm ahead)?" % v):
            continue
        input("  Strip ~40 cm ahead of the module, across the path. Enter.")
        tape = TapeSpec(mode="last_n", last_n_cm=80.0, miss_margin_cm=20.0)
        ex.run([g.Segment("F", "0", 0, 60.0)], leg="cal", speed=v, rolling=False, guard=False, tape=tape)
        time.sleep(0.5)
        d = ask_float("  Distance from the strip's centre line to the rear-axle centre now, cm")
        over = (offset + cm.rear_to_centre) - d
        print("  overshoot %.1f cm at %.0f cm/s" % (over, v))
        rows.append([v, round(over, 1)])
    if rows:
        cal["tape_overshoot"] = rows if len(rows) > 1 else [[0.0, 0.0]] + rows
    return True


CHECKS = {
    "api": check_api, "ultrasonic": check_ultrasonic, "motors": check_motors, "footprint": check_footprint,
    "steering_trim": check_steering_trim, "speed_table": check_speed_table,
    "radius_table": check_radius_table, "servo_swing": check_servo_swing,
    "grayscale_threshold": check_grayscale_threshold, "grayscale_offset": check_grayscale_offset,
}
assert tuple(CHECKS) == calibration.CHECKS


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", choices=calibration.CHECKS)
    args = ap.parse_args()
    from picarx import Picarx
    car = Picarx()
    cal = calibration.load()
    cal.setdefault("checks", {})
    names = [args.only] if args.only else list(calibration.CHECKS)
    ok_all = True
    ex = None
    try:
        for name in names:
            m = Motion(calibration.tables(cal))
            ex = Executor(car, m, ranger=ThreadedRanger(car, None), ramp_cm=calibration.car_model(cal).ramp_cm)
            ctx = {"car": car, "cal": cal, "m": m, "ex": ex}
            print("\n== %s" % name)
            ok = bool(CHECKS[name](ctx))
            cal["checks"][name] = ok
            calibration.save(cal)
            print("  ->", "PASS" if ok else "FAIL")
            if not ok:
                ok_all = False
                if not args.only:
                    print("Fix this before continuing.")
                    break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if ex is not None:
            ex.safe_shutdown()
        else:
            car.stop()
    print("\nchecks:", cal["checks"])
    res = calibration.resolve(cal)
    cap, why = res.speed_cap()
    print("Speed: %s" % ("full speed" if cap is None else
                         "run.py caps at %.0f %% until %s pass (or use --no-cap)" % (cap * 100, " and ".join(why))))
    for line in res.warnings():
        print(line)
    print("Next: copy calibration.json to the laptop and run plan.py; it must pass.")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
