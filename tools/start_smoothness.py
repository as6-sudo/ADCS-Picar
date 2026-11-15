#!/usr/bin/env python3
"""How abruptly does each leg of a profile start? (laptop, simulator)

    python tools/start_smoothness.py HL
    python tools/start_smoothness.py HH HL LH

For the first 0.4 s of motion of every leg (and every phase of the obstacle
event) it prints the peak acceleration, the peak turn rate and the speed
0.1 s / 0.3 s after the car starts moving, in the simulator with the default
calibration. Use it to compare how smooth a profile's starts look; this is
how HL's event start was found to be abrupt (48 deg/s from rest) and fixed
with a 5 cm lead-in.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration, paths  # noqa: E402
from trajectory_profiles.executor import Executor, Ranger  # noqa: E402
from trajectory_profiles.layout import DEFAULT as LAYOUT  # noqa: E402
from trajectory_profiles.motion import Motion  # noqa: E402
from trajectory_profiles.params import PROFILES as BEHAVIOURS  # noqa: E402
from trajectory_profiles.sim import ArenaSim  # noqa: E402

WINDOW_S = 0.4


def start_profile(segments, start, beh, car, motion):
    """(peak accel cm/s^2, peak turn deg/s, speed at +0.1 s, speed at +0.3 s)."""
    sim = ArenaSim(LAYOUT, car, start=start, strips=[])
    sim.place_obstacle((500, 500))                 # nothing in the way: only the motion matters
    ex = Executor(sim, motion, clock=sim, ranger=Ranger(sim, sim), ramp_cm=car.ramp_cm, log=lambda *a: None)
    samples = []
    step = sim._step

    def traced(dt):
        v0, h0 = sim.v, sim.phi
        step(dt)
        samples.append((sim.t, sim.v, (sim.v - v0) / dt, math.degrees(sim.phi - h0) / dt))
    sim._step = traced
    ex.run(segments, leg="x", speed=beh.cruise_cm_s, rolling=beh.rolling, ramp_fracs=beh.ramp_fracs, guard=False)
    moving = [s for s in samples if abs(s[1]) > 0.1]
    if not moving:
        return None
    t0 = moving[0][0]
    first = [s for s in samples if t0 <= s[0] <= t0 + WINDOW_S]

    def speed_at(dt):
        return next((abs(v) for t, v, _, _ in samples if t >= t0 + dt), 0.0)
    return (max(abs(a) for _, _, a, _ in first), max(abs(w) for _, _, _, w in first), speed_at(0.1), speed_at(0.3))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profiles", nargs="+", type=str.upper, choices=paths.PROFILE_KEYS)
    args = ap.parse_args(argv)
    car = calibration.DEFAULT_CAR
    motion = Motion(calibration.tables())
    for p in args.profiles:
        beh = BEHAVIOURS[p]
        print("\n%s (%s, %.0f cm/s)" % (p, beh.name, beh.cruise_cm_s))
        print("  %-48s %8s %8s %7s %7s" % ("leg / phase", "acc", "turn", "v+0.1", "v+0.3"))
        for leg in paths.legs(p, LAYOUT, car):
            pose = LAYOUT.target(leg.start)
            for ph in leg.phases:
                r = start_profile(ph.segments, pose, beh, car, motion)
                label = "%d %s" % (leg.number, leg.name if len(leg.phases) == 1 else "%s: %s" % (leg.name, ph.label))
                if r:
                    print("  %-48s %8.0f %8.0f %7.1f %7.1f" % ((label[:48],) + r))
                pose = paths.g.end_pose(pose, ph.segments, car.rear_to_centre)
    print("\nacc = peak acceleration (cm/s^2), turn = peak turn rate (deg/s) in the first %.1f s" % WINDOW_S)
    return 0


if __name__ == "__main__":
    sys.exit(main())
