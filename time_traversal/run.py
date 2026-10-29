#!/usr/bin/env python3
"""Launch a Time Traversal profile.

    python3 ~/picar-x/time_traversal/run.py A          # on the car
    python3 ~/picar-x/time_traversal/run.py D --sim    # no hardware
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from time_traversal.core import MockCar, home, run_profile, RUN_SECONDS  # noqa: E402
from time_traversal.profiles import PROFILES  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", type=str.upper, choices=sorted(PROFILES))
    ap.add_argument("--sim", action="store_true", help="use a mock car (no hardware)")
    ap.add_argument("--seed", type=int, default=None,
                    help="RNG seed for the random parts of profiles C and D")
    ap.add_argument("--duration", type=float, default=RUN_SECONDS)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    cls = PROFILES[args.profile]
    profile = cls(seed=args.seed)

    if args.sim:
        car = MockCar()
    else:
        from picarx import Picarx
        car = Picarx()
    print(f"Running profile {profile.key} ({profile.name}) for {args.duration:.0f}s")
    try:
        run_profile(profile, car, duration=args.duration, log=not args.quiet)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        home(car)


if __name__ == "__main__":
    main()
