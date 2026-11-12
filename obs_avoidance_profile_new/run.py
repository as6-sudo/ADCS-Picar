#!/usr/bin/env python3
"""Run one obstacle-avoidance behaviour profile (A, B, C or D).

    python3 ~/picar-x/obs_avoidance_profile_new/run.py A --sim     # laptop, no hardware
    python3 ~/picar-x/obs_avoidance_profile_new/calibrate.py       # on the car, first
    python3 ~/picar-x/obs_avoidance_profile_new/run.py C           # on the car

Emergency stop on the car: press Enter (or Ctrl+C). Motors stop and servos
centre on any error or interruption.

Until every calibrate.py check has passed, hardware runs are capped at
--speed-scale 0.5 and profile D (closest approach) is refused.
"""
import argparse
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from obs_avoidance_profile_new import calibration  # noqa: E402
from obs_avoidance_profile_new.params import PROFILES, UNCALIBRATED_SPEED_SCALE  # noqa: E402
from obs_avoidance_profile_new.robot import EStop, RealClock, Robot  # noqa: E402
from obs_avoidance_profile_new.runner import run_profile  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", type=str.upper, choices=sorted(PROFILES))
    ap.add_argument("--sim", action="store_true", help="simulated car (no hardware)")
    ap.add_argument("--speed-scale", type=float, default=None,
                    help="multiply every speed (default 1.0 if calibrated, else 0.5)")
    ap.add_argument("--obstacle-cm", type=float, default=120.0, help="sim: start distance")
    ap.add_argument("--seed", type=int, default=0, help="sim: sensor-noise seed")
    args = ap.parse_args(argv)

    P = PROFILES[args.profile]
    cal = calibration.load()
    estop = EStop()

    if args.sim:
        from obs_avoidance_profile_new.sim import SimCar
        car = SimCar(obstacle_cm=args.obstacle_cm, seed=args.seed)
        clock = car
        scale = args.speed_scale if args.speed_scale is not None else 1.0
    else:
        if P.key == "D" and not calibration.passed(cal, "stop_distance"):
            sys.exit("Profile D approaches to 20 cm: run calibrate.py and pass the "
                     "stop_distance check first.")
        scale = args.speed_scale if args.speed_scale is not None else 1.0
        if not calibration.all_passed(cal) and scale > UNCALIBRATED_SPEED_SCALE:
            print("Calibration incomplete -> speeds capped at %.0f%%. Run calibrate.py."
                  % (UNCALIBRATED_SPEED_SCALE * 100))
            scale = UNCALIBRATED_SPEED_SCALE
        from picarx import Picarx
        car = Picarx()
        clock = RealClock()
        signal.signal(signal.SIGTERM, lambda *_: estop.trigger("SIGTERM"))
        if sys.stdin.isatty():
            estop.listen_for_enter()
            print(">>> Press Enter at any time for an EMERGENCY STOP <<<")

    robot = Robot(car, clock=clock, estop=estop, speed_scale=scale)
    max_speed = calibration.max_safe_speed(cal)
    print("Profile %s (%s), speed scale %.2f, max safe speed %d%s"
          % (P.key, P.name, scale, max_speed, " [SIM]" if args.sim else ""))
    try:
        ok = run_profile(robot, P, max_speed)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl+C): motors stopped.")
        ok = False
    finally:
        robot.safe_shutdown()
    if args.sim:
        print("SIM: min clearance %.1f cm, collided=%s, turned %.0f deg clockwise, "
              "moved x=%+.0f cm y=%+.0f cm"
              % (car.min_clearance, car.collided, car.heading_deg_cw(), car.x, car.y))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
