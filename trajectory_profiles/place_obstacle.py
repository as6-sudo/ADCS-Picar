#!/usr/bin/env python3
"""Live ultrasonic readout for placing the obstacle at O0. The car never moves.

Put the car at A facing the targets, run this, and slide the obstacle along the
car's centre line until it says OK. Ctrl+C to quit, then start run.py without
touching the car.

    python3 trajectory_profiles/place_obstacle.py            # target from layout + calibration
    python3 trajectory_profiles/place_obstacle.py --tol 1.0

The target is the distance from the ultrasonic face to the obstacle's near
face when the car's centre is at A and the obstacle's centre is at O0:
(O0.x - obstacle_half) - A.x - car.ultrasonic_offset.
"""
import argparse
import os
import statistics
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration  # noqa: E402
from trajectory_profiles.layout import Layout  # noqa: E402


def target_cm(layout=None, cal=None):
    layout = layout or Layout()
    car = calibration.car_model(cal if cal is not None else calibration.load())
    a = layout.targets["A"]
    return (layout.o0[0] - layout.obstacle_half) - a.x - car.ultrasonic_offset


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", type=float, default=None, help="override the target distance, cm")
    ap.add_argument("--tol", type=float, default=1.5, help="OK band, +/- cm")
    args = ap.parse_args()
    target = args.target if args.target is not None else target_cm()

    from picarx import Picarx
    car = Picarx()
    car.stop()
    car.set_dir_servo_angle(0)
    print("Car stays still. Obstacle near face should be %.1f cm from the ultrasonic (+/- %.1f)."
          % (target, args.tol))
    print("Slide the obstacle along the car's centre line. Ctrl+C when it says OK.\n")
    recent = deque(maxlen=7)
    try:
        while True:
            try:
                d = float(car.get_distance())
            except (TypeError, ValueError):
                d = -1
            if 2.0 <= d <= 400.0:
                recent.append(d)
            if len(recent) >= 3:
                m = statistics.median(recent)
                err = m - target
                if abs(err) <= args.tol:
                    msg = "OK  (%+.1f cm)" % err
                elif err > 0:
                    msg = "move it %4.1f cm CLOSER" % err
                else:
                    msg = "move it %4.1f cm FURTHER" % -err
                line = "target %5.1f cm   now %6.1f cm   %s" % (target, m, msg)
            else:
                line = "waiting for valid readings..."
            sys.stdout.write("\r" + line.ljust(60))
            sys.stdout.flush()
            time.sleep(0.08)
    except KeyboardInterrupt:
        print("\nDone. Leave the car where it is and start run.py.")
    finally:
        car.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
