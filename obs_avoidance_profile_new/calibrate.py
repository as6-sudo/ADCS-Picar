#!/usr/bin/env python3
"""Step-by-step hardware checks. Run ON THE CAR before any full profile.

    python3 ~/picar-x/obs_avoidance_profile_new/calibrate.py          # all checks
    python3 ~/picar-x/obs_avoidance_profile_new/calibrate.py --only ultrasonic

Checks, in order (each asks you to confirm what you saw):
  api            the installed picarx has every method/limit these profiles use
  ultrasonic     30 readings at a known distance; validity rate and spread
  motors         short forward then reverse blip at low speed (wheels on the floor, 50 cm clear)
  camera_pan     head goes LEFT, then RIGHT, then centre
  steering       wheels point LEFT, then RIGHT, then centre; optional short right arc
  stop_distance  drive slowly at an obstacle; record where it actually stops,
                 then optionally repeat at the speed you want profile B to use

Results are stored in calibration.json next to this file. run.py caps speeds
at 50 % until all checks pass, and refuses profile D until stop_distance passes.
Emergency stop: Ctrl+C (motors stop, servos centre).
"""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from obs_avoidance_profile_new import calibration  # noqa: E402
from obs_avoidance_profile_new.params import (APPROACH_TIMEOUT_S, DETECT_CM,  # noqa: E402
                                              HARD_MIN_CM)
from obs_avoidance_profile_new.robot import ProfileAbort, Robot, validate_distance  # noqa: E402

REQUIRED_METHODS = ("forward", "backward", "stop", "set_dir_servo_angle",
                    "set_cam_pan_angle", "set_cam_tilt_angle", "get_distance")
TEST_SPEED = 30


def ask(q):
    return input(q + " [y/n] ").strip().lower().startswith("y")


def check_api(car, r):
    missing = [m for m in REQUIRED_METHODS if not callable(getattr(car, m, None))]
    limits = {k: getattr(car, k, None) for k in ("DIR_MIN", "DIR_MAX", "CAM_PAN_MIN", "CAM_PAN_MAX")}
    print("  methods missing:", missing or "none", "| limits:", limits)
    return not missing and limits["DIR_MAX"] == 30 and limits["CAM_PAN_MAX"] >= 60


def check_ultrasonic(car, r):
    input("  Place a flat object 30 cm in front of the sensor, then press Enter.")
    raw = []
    for _ in range(30):
        raw.append(car.get_distance())
        time.sleep(0.05)
    ok = [d for d in map(validate_distance, raw) if d is not None]
    print("  raw:", raw)
    if not ok:
        print("  no valid readings")
        return False
    print("  valid %d/30  median %.1f  min %.1f  max %.1f cm"
          % (len(ok), statistics.median(ok), min(ok), max(ok)))
    return len(ok) >= 24 and 25 <= statistics.median(ok) <= 35 and ask("  Does that match ~30 cm?")


def check_motors(car, r):
    input("  Wheels on the floor, 50 cm clear front and back. Press Enter.")
    r.move(+1, TEST_SPEED, 0.5)
    time.sleep(0.5)
    r.move(-1, TEST_SPEED, 0.5)
    return ask("  Did it go FORWARD first, then BACKWARD?")


def check_camera_pan(car, r):
    for label, a in (("LEFT", -45), ("RIGHT", 45), ("centre", 0)):
        print("  head ->", label)
        r.pan_to(a, 90)
        time.sleep(0.6)
    return ask("  Did the camera head look LEFT first, then RIGHT, and end centred?")


def check_steering(car, r):
    for label, a in (("LEFT", -30), ("RIGHT", 30), ("centre", 0)):
        print("  wheels ->", label)
        r.set_steer(a)
        time.sleep(0.8)
    if not ask("  Did the front wheels point LEFT first, then RIGHT, and end straight?"):
        return False
    if ask("  Drive a short RIGHT arc (1 s at speed %d)? Needs ~60 cm clear." % TEST_SPEED):
        r.move(+1, TEST_SPEED, 1.0, steer=30)
        r.set_steer(0)
        return ask("  Did it curve to the RIGHT?")
    return True


def stop_test(r, speed):
    input("  Put the obstacle ~1 m straight ahead, then press Enter.")
    try:
        trig = r.approach(speed, DETECT_CM, APPROACH_TIMEOUT_S)
    except ProfileAbort as e:
        print("  ", e)
        return None
    time.sleep(0.5)
    finals = [d for d in (validate_distance(r.car.get_distance()) for _ in range(5)) if d]
    final = statistics.median(finals) if finals else float("nan")
    print("  speed %d: triggered at %.1f cm, came to rest at %.1f cm (threshold %.0f)"
          % (speed, trig, final, DETECT_CM))
    return final


def check_stop_distance(car, r, cal):
    final = stop_test(r, TEST_SPEED)
    if final is None or not final >= HARD_MIN_CM + 5:
        print("  stopped too close (or no reading): raise DETECT_CM in params.py")
        return False
    cal["stop_cm_at_test_speed"] = final
    if ask("  Also test a faster speed for profile B?"):
        speed = int(input("  speed (40-100): "))
        f = stop_test(r, speed)
        if f is not None and f >= HARD_MIN_CM + 5 and ask("  Use %d as the max safe speed?" % speed):
            cal["max_safe_speed"] = speed
    return ask("  Did it stop clearly before the obstacle?")


CHECKS = {
    "api": check_api, "ultrasonic": check_ultrasonic, "motors": check_motors,
    "camera_pan": check_camera_pan, "steering": check_steering,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", choices=calibration.CHECKS)
    args = ap.parse_args()
    from picarx import Picarx
    car = Picarx()
    r = Robot(car, log=print)
    cal = calibration.load()
    cal.setdefault("checks", {})
    names = [args.only] if args.only else list(calibration.CHECKS)
    try:
        r.center()
        for name in names:
            print("\n== %s" % name)
            if name == "stop_distance":
                ok = check_stop_distance(car, r, cal)
            else:
                ok = CHECKS[name](car, r)
            cal["checks"][name] = bool(ok)
            print("  ->", "PASS" if ok else "FAIL")
            calibration.save(cal)
            if not ok and not args.only:
                print("Fix this before continuing.")
                break
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        r.safe_shutdown()
    print("\nchecks:", cal["checks"], "| max_safe_speed:", calibration.max_safe_speed(cal))
    print("All passed -> full speed enabled." if calibration.all_passed(cal)
          else "Not all passed -> run.py stays at reduced speed.")


if __name__ == "__main__":
    main()
