#!/usr/bin/env python3
"""Repeatability: run one leg N times and measure where the car lands.

    python3 trajectory_profiles/repeat.py HH 1            # leg 1 of HH, 5 runs (on the car)
    python3 trajectory_profiles/repeat.py HL 4 -n 8
    python trajectory_profiles/repeat.py HH 3 --sim       # simulator, 5 noise seeds

After each run, type where the car's CENTRE landed, either as arena
coordinates "x y" (cm) or as offsets from the target X: "d dx dy".
The tool prints the mean, the spread ALONG the direction of travel and
SIDEWAYS, and a recommendation (brief 9.4):
  * spread within 3 cm both ways  -> tape the X at the mean position;
  * spread mostly along travel    -> enable the tape stop for this target;
  * spread mostly sideways        -> tune the curve (calibration / radius table).
"""
import argparse
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration, paths, profiles  # noqa: E402
from trajectory_profiles.executor import Executor, Ranger, ThreadedRanger  # noqa: E402
from trajectory_profiles.layout import DEFAULT as LAYOUT  # noqa: E402
from trajectory_profiles.motion import Motion  # noqa: E402

SPREAD_OK_CM = 3.0           # brief 9.4


def analyse(points, target, travel_heading_deg=0.0, ok_cm=SPREAD_OK_CM):
    """points: [(x, y)] landing positions of the car centre. Returns a dict.

    'along' is the direction of travel at the target (heading 0 = +x for
    every target in this layout), 'side' is perpendicular to it. Spreads
    are max - min (the range), which is what the eye sees on camera."""
    h = math.radians(travel_heading_deg)
    ux, uy = math.cos(h), math.sin(h)
    along = [(x - target.x) * ux + (y - target.y) * uy for x, y in points]
    side = [-(x - target.x) * uy + (y - target.y) * ux for x, y in points]
    mx = statistics.mean(x for x, _ in points)
    my = statistics.mean(y for _, y in points)
    a_spread = max(along) - min(along)
    s_spread = max(side) - min(side)
    if a_spread <= ok_cm and s_spread <= ok_cm:
        rec = "tape the X at (%.1f, %.1f)" % (mx, my)
    elif a_spread >= s_spread:
        rec = "enable the tape stop for this target (spread is mostly along the direction of travel)"
    else:
        rec = "tune the curve: recheck steering_trim / radius_table (spread is mostly sideways)"
    return dict(mean=(mx, my), mean_along=statistics.mean(along), mean_side=statistics.mean(side),
                along_spread=a_spread, side_spread=s_spread,
                along_sd=statistics.pstdev(along), side_sd=statistics.pstdev(side), recommendation=rec)


def parse_landing(text, target):
    parts = text.split()
    if parts and parts[0].lower() == "d":
        return target.x + float(parts[1]), target.y + float(parts[2])
    return float(parts[0]), float(parts[1])


def report(res, target_name, n):
    print("\n%d runs, target %s" % (n, target_name))
    print("  mean landing   (%.1f, %.1f)   offset along %+.1f cm, sideways %+.1f cm (+ = left)"
          % (res["mean"] + (res["mean_along"], res["mean_side"])))
    print("  spread along   %.1f cm (sd %.1f)" % (res["along_spread"], res["along_sd"]))
    print("  spread sideways %.1f cm (sd %.1f)" % (res["side_spread"], res["side_sd"]))
    print("  -> " + res["recommendation"])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", type=str.upper, choices=("HH", "HL", "LH"))
    ap.add_argument("leg", type=int)
    ap.add_argument("-n", type=int, default=5, help="number of runs (default 5)")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--speed-scale", type=float, default=None)
    args = ap.parse_args(argv)

    cal = calibration.load()
    car, motion, layout = calibration.car_model(cal), Motion(calibration.tables(cal)), LAYOUT
    leg = paths.legs(args.profile, layout, car)[args.leg - 1]
    target = layout.target(leg.end)
    scale = args.speed_scale if args.speed_scale is not None else 1.0
    if not args.sim and not calibration.all_passed(cal):
        scale = min(scale, 0.5)
    points = []
    hw = None
    if not args.sim:
        from picarx import Picarx
        from trajectory_profiles.executor import RealClock
        hw = Picarx()
    for i in range(args.n):
        if args.sim:
            from trajectory_profiles.sim import ArenaSim
            hw = ArenaSim(layout, car, start=layout.target(leg.start), seed=i, tables=calibration.tables(cal),
                          strips=list(profiles.strips(args.profile, layout, car, motion).values()))
            clock, ranger = hw, Ranger(hw, hw)
        else:
            input("\nRun %d/%d: place the car at %s (facing +x), obstacle where the leg needs it. Enter."
                  % (i + 1, args.n, leg.start))
            clock, ranger = RealClock(), ThreadedRanger(hw, None)
        ex = Executor(hw, motion, clock=clock, ranger=ranger, ramp_cm=car.ramp_cm, log=lambda *a: None)
        take = profiles.Take(args.profile, ex, layout, car, motion, speed_scale=scale,
                             sim=hw if args.sim else None, log=lambda *a: None, cue_out=lambda: None)
        ok = take.run(only_leg=args.leg)
        if not ok:
            print("  run %d stopped early; not counted" % (i + 1))
            continue
        if args.sim:
            p = hw.pose()
            points.append((p.x, p.y))
            print("  run %d: (%.1f, %.1f)" % (i + 1, p.x, p.y))
        else:
            while True:
                try:
                    points.append(parse_landing(input("  landing of the car centre ('x y' or 'd dx dy'): "), target))
                    break
                except (ValueError, IndexError):
                    print("  e.g. '124.5 36' or 'd -0.5 1'")
    if len(points) < 2:
        print("not enough runs")
        return 1
    report(analyse(points, target, target.h), leg.end, len(points))
    return 0


if __name__ == "__main__":
    sys.exit(main())
