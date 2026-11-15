#!/usr/bin/env python3
"""Run one profile take (or one leg) on the car or in the simulator.

    python3 trajectory_profiles/run.py HH --dry-run       # segment table + tape positions, nothing moves
    python3 trajectory_profiles/run.py HH --sim           # whole take in the simulator (laptop)
    python3 trajectory_profiles/run.py HH --leg 3         # one leg on the car (place it at the leg's start)
    python3 trajectory_profiles/run.py LH --speed-scale 0.5

On the car, run it over SSH from the operator's laptop: the PULL cue (bell +
big text) appears in the laptop terminal. The robot itself never beeps.

Emergency stop: Enter, Ctrl+C or SIGTERM. The motors stop and every servo
centres on any error, abort or interrupt. Reverse moves are UNGUARDED (no
rear sensor): keep the area behind A clear.

Refuses to run if plan.py's checks fail for the current car model. Until
every calibrate.py check passes, speeds are capped at 50 %.
Every segment and sensor event goes to logs/<profile>_<timestamp>.csv.
"""
import argparse
import csv
import datetime
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration, paths, profiles  # noqa: E402
from trajectory_profiles.executor import EStop, Executor, Ranger, RealClock, ThreadedRanger  # noqa: E402
from trajectory_profiles.layout import DEFAULT as LAYOUT  # noqa: E402
from trajectory_profiles.motion import Motion, check_radius_consistency  # noqa: E402
from trajectory_profiles.params import PROFILES as BEHAVIOURS  # noqa: E402
from trajectory_profiles.plan import check_profile  # noqa: E402

UNCALIBRATED_SPEED_SCALE = 0.5      # brief 6.6: 50 % until calibration has passed
HERE = os.path.dirname(os.path.abspath(__file__))
FIELDS = ["t", "kind", "leg", "index", "segment", "driven", "steer", "cmd", "planned_s", "actual_s",
          "event", "detail"]


class CsvLog:
    def __init__(self, path, clock):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.DictWriter(self.f, FIELDS, extrasaction="ignore")
        self.w.writeheader()
        self.clock, self.t0 = clock, clock.now()
        self.path = path

    def __call__(self, **kw):
        extra = {k: v for k, v in kw.items() if k not in FIELDS}
        row = dict(kw, t="%.3f" % (self.clock.now() - self.t0))
        if extra:
            row["detail"] = (str(row.get("detail", "")) + " " + " ".join("%s=%s" % i for i in extra.items())).strip()
        self.w.writerow(row)
        self.f.flush()

    def close(self):
        self.f.close()


def segment_table(profile, layout, car, motion, speed_scale):
    """Rows for --dry-run: what each segment will be commanded to do."""
    beh = BEHAVIOURS[profile]
    v = beh.cruise_cm_s * speed_scale
    rows = []
    for leg in paths.legs(profile, layout, car):
        for ph in leg.phases:
            for s in ph.segments:
                steer = motion.steer_for(s.side, s.radius)
                cmd = motion.command_for(v, steer)
                gv = motion.ground_speed(cmd, steer)
                rows.append((leg.number, ph.label, str(s), steer, cmd, gv, s.length / gv))
    return rows


def dry_run(profile, layout, car, motion, scale):
    print("%-4s %-34s %-16s %7s %4s %7s %6s" % ("leg", "phase", "segment", "steer", "cmd", "cm/s", "s"))
    total = 0.0
    for leg, ph, s, steer, cmd, v, t in segment_table(profile, layout, car, motion, scale):
        print("%-4d %-34s %-16s %+7.1f %4d %7.1f %6.2f" % (leg, ph[:34], s, steer, cmd, v, t))
        total += t
    print("driving time ~%.1f s (without ramps, pauses and gazes)" % total)
    print("\nTape strips for %s (centre line x; strips %.1f cm wide along x, %.0f cm long along y):"
          % (profile, layout.strip_width_cm, layout.strip_length_cm))
    for name, (x0, x1, y0, y1) in profiles.strips(profile, layout, car, motion).items():
        print("  %s: x = %.1f, y = %.1f .. %.1f" % (name, (x0 + x1) / 2, y0, y1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profile", type=str.upper, choices=profiles.PROFILE_KEYS)
    ap.add_argument("--leg", type=int, help="run only this leg (car placed at its start)")
    ap.add_argument("--sim", action="store_true", help="simulated car (no hardware)")
    ap.add_argument("--dry-run", action="store_true", help="print the segment table only")
    ap.add_argument("--speed-scale", type=float, default=None,
                    help="multiply every speed (default 1.0 if calibrated, else 0.5)")
    ap.add_argument("--seed", type=int, default=0, help="sim: sensor-noise seed")
    ap.add_argument("--late", action="store_true", help="sim: nobody pulls the line (tests OBSTACLE_LATE / NO_OBSTACLE)")
    args = ap.parse_args(argv)

    if args.profile == "LL":
        raise NotImplementedError("Profile LL is out of scope for this framework (brief section 1).")

    cal = calibration.load()
    car = calibration.car_model(cal)
    motion = Motion(calibration.tables(cal))
    layout = LAYOUT

    scale = args.speed_scale if args.speed_scale is not None else 1.0
    if not args.sim and not calibration.all_passed(cal) and scale > UNCALIBRATED_SPEED_SCALE:
        print("Calibration incomplete -> speeds capped at %.0f%%. Run calibrate.py."
              % (UNCALIBRATED_SPEED_SCALE * 100))
        scale = UNCALIBRATED_SPEED_SCALE

    rep = check_profile(args.profile, layout, car)
    problems = rep.fails + check_radius_consistency(motion, car)
    if problems:
        print("plan.py checks FAILED for this car model; refusing to run:")
        for f in problems:
            print("  " + f)
        return 2
    for w in rep.warnings:
        print("note: " + w)

    if args.dry_run:
        dry_run(args.profile, layout, car, motion, scale)
        return 0

    estop = EStop()
    if args.sim:
        from trajectory_profiles.sim import ArenaSim
        start = layout.target(paths.legs(args.profile, layout, car)[args.leg - 1].start) if args.leg else None
        sim = ArenaSim(layout, car, start=start, tables=calibration.tables(cal), seed=args.seed,
                       strips=list(profiles.strips(args.profile, layout, car, motion).values()))
        if args.late:
            sim.start_pull = lambda delay=0.0: None
        car_hw, clock, ranger = sim, sim, Ranger(sim, sim)
    else:
        from picarx import Picarx
        car_hw, clock, sim = Picarx(), RealClock(), None
        ranger = ThreadedRanger(car_hw, clock)
        signal.signal(signal.SIGTERM, lambda *_: estop.trigger("SIGTERM"))
        if sys.stdin.isatty():
            estop.listen_for_enter()
            print(">>> Press Enter at any time for an EMERGENCY STOP <<<")

    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log = CsvLog(os.path.join(HERE, "logs", "%s%s_%s%s.csv" % (
        args.profile, "_leg%d" % args.leg if args.leg else "", stamp, "_sim" if args.sim else "")), clock)
    ex = Executor(car_hw, motion, clock=clock, estop=estop, ranger=ranger, record=log, ramp_cm=car.ramp_cm)
    take = profiles.Take(args.profile, ex, layout, car, motion, speed_scale=scale, sim=sim)
    print("Profile %s (%s), speed scale %.2f%s" % (args.profile, BEHAVIOURS[args.profile].name, scale,
                                                    " [SIM]" if args.sim else ""))
    try:
        ok = take.run(only_leg=args.leg)
    except KeyboardInterrupt:
        print("Interrupted (Ctrl+C): motors stopped.")
        ok = False
    finally:
        ex.safe_shutdown()
        log.close()
    flagged = [e for e in take.events if e[1] in ("OBSTACLE_LATE", "TAPE_MISSED", "NO_OBSTACLE", "ESTOP", "ABORT")]
    for leg, name, detail in flagged:
        print("  !! %s %s %s" % (leg, name, detail))
    if args.sim:
        p = sim.pose()
        print("SIM: end pose (%.1f, %.1f, %.1f deg), min obstacle clearance %.1f cm, min edge %.1f cm, "
              "collided=%s, %.1f s" % (p.x, p.y, p.h, sim.min_obstacle_clearance, sim.min_edge_clearance,
                                       sim.collided, sim.t))
    print("log: %s" % log.path)
    print("%s" % ("take completed" if ok and not flagged else "take NOT clean: redo it" if ok else "take stopped"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
