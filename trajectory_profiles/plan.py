#!/usr/bin/env python3
"""Check every leg of every profile in the ideal bicycle model (laptop).

    python trajectory_profiles/plan.py                 # all profiles, table + verdict
    python trajectory_profiles/plan.py HH --segments   # also print each leg's segments
    python trajectory_profiles/plan.py --png           # also save plots/<profile>.png (needs matplotlib)
    python trajectory_profiles/plan.py --solve         # re-solve MAIN/ALT/CLOSE/event for this layout + car

Checks per leg: end pose on target (< 0.5 cm, < 0.5 deg), obstacle clearance
(O0 before the event, O1 after, the swept strip while the obstacle slides),
arena-edge clearance, radii >= the calibrated minimum, tape-stop geometry,
and (as warnings) room for speed ramps. The car model comes from
calibration.json if it exists, so a measured car that breaks a rule is
reported here and refused by run.py; nothing runs silently.
Exit code 1 if any check fails.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration, geometry as g, paths, solver  # noqa: E402
from trajectory_profiles.layout import DEFAULT as DEFAULT_LAYOUT  # noqa: E402
from trajectory_profiles.params import PROFILES as BEHAVIOURS  # noqa: E402

STEP = 0.5          # cm between checked poses


class Report:
    def __init__(self, profile):
        self.profile = profile
        self.rows = []          # dicts for the table
        self.fails = []
        self.warnings = []

    @property
    def ok(self):
        return not self.fails


def check_profile(profile, layout=DEFAULT_LAYOUT, car=None):
    car = car or calibration.car_model(calibration.load())
    beh = BEHAVIOURS[profile]
    rep = Report(profile)
    off = car.rear_to_centre
    for leg in paths.legs(profile, layout, car):
        tag = "%s leg %d" % (profile, leg.number)
        pose = layout.target(leg.start)
        for ph in leg.phases:
            poses = [p for p, _ in g.sample(pose, ph.segments, off, STEP)]
            obs = paths.obstacle_polygons(ph.obstacle, layout)
            clear = min((g.clearance(poses, o, car) for o in obs), default=float("inf"))
            need = layout.min_clearance[ph.kind]
            edge = g.edge_clearance(poses, car, layout.width, layout.height)
            margin = layout.edge_margin if ph.edge_margin is None else ph.edge_margin
            status = "ok"
            if clear < need:
                status = "FAIL"
                rep.fails.append("%s %s: %.1f cm to %s < %.1f" % (tag, ph.label, clear, ph.obstacle, need))
            if edge < margin:
                status = "FAIL"
                rep.fails.append("%s %s: %.1f cm to the arena edge < %.1f" % (tag, ph.label, edge, margin))
            rep.rows.append(dict(leg=leg.number, name=leg.name, phase=ph.label, obstacle=ph.obstacle,
                                 clear=clear, need=need, edge=edge, margin=margin, status=status,
                                 segments=ph.segments, end=None))
            pose = g.end_pose(pose, ph.segments, off)

        # end pose
        target = layout.target(leg.end)
        dpos, dh = g.pose_error(pose, target)
        rep.rows[-1]["end"] = (dpos, dh)
        if dpos > layout.end_tolerance_cm or dh > layout.end_tolerance_deg:
            rep.rows[-1]["status"] = "FAIL"
            rep.fails.append("%s ends %.2f cm / %.2f deg from %s" % (tag, dpos, dh, leg.end))

        # planned-but-never-driven parts (documentation): checked against O0
        for start, s in leg.planned:
            if leg.planned_kind:
                poses = [p for p, _ in g.sample(start, s, off, STEP)]
                c = g.clearance(poses, paths.obstacle_polygons("O0", layout)[0], car)
                need = layout.min_clearance[leg.planned_kind]
                if c < need:
                    rep.warnings.append("%s planned (undriven) path: %.1f cm to O0 < %.1f" % (tag, c, need))

        # radii
        for s in leg.segments:
            if s.is_arc and s.radius < car.min_radius - 1e-6:
                rep.fails.append("%s uses R%g < calibrated minimum radius %.1f" % (tag, s.radius, car.min_radius))

        # tape stop at the target
        tape = layout.tape[leg.end]
        last = leg.segments[-1]
        if tape.enabled and tape.mode == "final_straight" and last.is_arc:
            rep.fails.append("%s ends on an arc: the final-straight tape stop at %s would never arm "
                             "(set mode='last_n' for %s)" % (tag, leg.end, leg.end))

        # ramps (warnings: the leg still runs, just without that ramp)
        if beh.ramps:
            first = leg.segments[0]
            if first.is_arc or first.length < car.ramp_cm:
                rep.warnings.append("%s starts with %s: no %.0f cm straight to ramp up on, starts at constant speed"
                                    % (tag, first, car.ramp_cm))
            if last.is_arc or last.length < car.ramp_cm:
                rep.warnings.append("%s ends with %s: shorter than the %.0f cm ramp, slows down on the last arc"
                                    % (tag, last, car.ramp_cm))

        # event: ultrasonic confirmation at the stop
        if leg.name.endswith("event"):
            stop = g.end_pose(layout.target("A"), leg.phases[0].segments, off)
            d = paths.ultrasonic_reading(stop, layout, car, "O1")
            rep.rows[-1]["note"] = "stop at (%.1f, %.1f, %.0f deg), sensor ~%s cm" % (
                stop.x, stop.y, stop.h, "%.1f" % d if d else "none")
            if d is None or d > layout.event_late_cm:
                rep.warnings.append("%s: ultrasonic at the stop reads %s, above the %.0f cm OBSTACLE_LATE "
                                    "threshold even when the obstacle is on time" % (tag, d, layout.event_late_cm))
    return rep


def print_report(rep, show_segments=False):
    print("\n=== %s (%s)" % (rep.profile, BEHAVIOURS[rep.profile].name))
    print("%-3s %-40s %-34s %6s %8s %7s %7s %-6s %s" % (
        "leg", "name", "phase", "obst", "clear", "edge", "end cm", "", ""))
    for r in rep.rows:
        end = "%.2f" % r["end"][0] if r["end"] else ""
        print("%-3d %-40s %-34s %6s %5.1f/%-2.0f %7.1f %7s %-6s %s" % (
            r["leg"], r["name"][:40], r["phase"][:34], r["obstacle"], r["clear"], r["need"], r["edge"],
            end, r["status"], r.get("note", "")))
        if show_segments:
            print("      " + g.fmt(r["segments"]))
    for w in rep.warnings:
        print("  warning: " + w)
    for f in rep.fails:
        print("  FAIL: " + f)


# ------------------------------------------------------------------ plotting
COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]


def plot_profile(profile, layout, car, path):
    """Top-down PNG in the style of the storyboards. matplotlib is optional
    and only imported here."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    off = car.rear_to_centre
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.add_patch(Rectangle((0, 0), layout.width, layout.height, fill=False, lw=2, color="k"))
    h = layout.obstacle_half
    sweep = paths.obstacle_polygons("SWEEP", layout)[0]
    ax.add_patch(Polygon(sweep, closed=True, color="0.85", lw=0, zorder=0))
    ax.add_patch(Rectangle((layout.o0[0] - h, layout.o0[1] - h), 2 * h, 2 * h, color="0.3", alpha=.8, label="O0"))
    ax.add_patch(Rectangle((layout.o1[0] - h, layout.o1[1] - h), 2 * h, 2 * h, fill=False, ls="--",
                           color="0.3", label="O1"))
    for name, t in layout.targets.items():
        ax.plot(t.x, t.y, "kx", ms=10, mew=2)
        ax.annotate(name, (t.x, t.y), textcoords="offset points", xytext=(6, 6), fontsize=12, weight="bold")
        if layout.tape[name].enabled:
            sx = t.x + car.grayscale_offset
            ax.plot([sx, sx], [t.y - 6, t.y + 6], color="k", lw=4, alpha=.6)
    for leg in paths.legs(profile, layout, car):
        col = COLORS[(leg.number - 1) % len(COLORS)]
        for start, s in leg.planned:
            pts = [p for p, _ in g.sample(start, s, off, 1.0)]
            ax.plot([p.x for p in pts], [p.y for p in pts], ls=":", color=col, lw=1)
        pose = layout.target(leg.start)
        first = True
        for ph in leg.phases:
            pts = [p for p, _ in g.sample(pose, ph.segments, off, 1.0)]
            ax.plot([p.x for p in pts], [p.y for p in pts], color=col, lw=2,
                    label="%d %s" % (leg.number, leg.name) if first else None)
            first = False
            for p in pts[::25]:
                ax.add_patch(Polygon(g.footprint(p, car), closed=True, fill=False, color=col, alpha=.25, lw=.8))
            pose = g.end_pose(pose, ph.segments, off)
    ax.set_xlim(-5, layout.width + 5)
    ax.set_ylim(-5, layout.height + 5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title("%s: %s" % (profile, BEHAVIOURS[profile].name))
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ------------------------------------------------------------------ solver front-end
def run_solver(layout, car):
    A, B, C = layout.target("A"), layout.target("B"), layout.target("C")
    O0 = paths.obstacle_polygons("O0", layout)[0]
    O1 = paths.obstacle_polygons("O1", layout)[0]
    off = car.rear_to_centre

    def show(name, cand, current):
        print("\n%s" % name)
        print("  current: " + g.fmt(current))
        if cand is None:
            print("  solved : NO SOLUTION")
            return
        print("  solved : " + g.fmt(cand.segments))
        print("           clearance %.1f cm, edge %.1f cm" % (cand.clearance, cand.edge))

    mc = layout.min_clearance
    main = solver.solve(A, B, O0, car, layout, side1="R", lead_in=car.ramp_cm, min_clear=mc["MAIN"])
    show("MAIN (lead-in = ramp %.1f cm, >= %.0f cm to O0)" % (car.ramp_cm, mc["MAIN"]), main, paths.MAIN)
    if main is None:
        fb = solver.solve(A, B, O0, car, layout, side1="R", lead_in=0, min_clear=mc["MAIN"])
        show("MAIN fallback (no lead-in: ramp up during the pull-out arc, calibrate the ramp-in-arc distance)",
             fb, paths.MAIN_BRIEF)
    show("ALT (lead-in max(5, ramp), %.0f..%.0f cm to O0)" % (mc["ALT"], layout.max_clearance["ALT"]),
         solver.solve(A, B, O0, car, layout, side1="R", lead_in=max(5.0, car.ramp_cm), min_clear=mc["ALT"],
                      max_clear=layout.max_clearance["ALT"]),
         paths.ALT)
    show("CLOSE (lead-in max(10, ramp), %.0f..%.0f cm to O0)" % (mc["CLOSE"], layout.max_clearance["CLOSE"]),
         solver.solve(A, B, O0, car, layout, side1="R", lead_in=max(10.0, car.ramp_cm), min_clear=mc["CLOSE"],
                      max_clear=layout.max_clearance["CLOSE"]),
         paths.CLOSE)
    backup = g.end_pose(g.end_pose(A, paths.event_approach(layout.event_stop_travel_cm), off),
                        paths.EVENT_BACKUP, off)
    show("EVENT re-plan from (%.1f, %.1f, %.0f deg), >= %.0f cm to O1" % (backup.x, backup.y, backup.h, mc["EVENT"]),
         solver.solve(backup, C, O1, car, layout, side1="L", lead_in=0, min_clear=mc["EVENT"],
                      angles=solver.frange(16, 45, 1), r1s=solver.frange(car.min_radius, 60, 5),
                      r2s=solver.frange(60, 250, 10), max_mid=40.0),
         paths.EVENT_REPLAN)
    print("\nThe solver prints; it never edits paths.py. Paste a solution in by hand, then rerun plan.py.")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("profiles", nargs="*", type=str.upper, choices=list(paths.PROFILE_KEYS),
                    help="default: all")
    ap.add_argument("--segments", action="store_true", help="print each phase's segments")
    ap.add_argument("--png", action="store_true", help="save a top-down plot per profile")
    ap.add_argument("--solve", action="store_true", help="re-solve the curves for this layout and car")
    ap.add_argument("--calibration", default=calibration.PATH, help="calibration.json to take the car model from")
    args = ap.parse_args(argv)

    car = calibration.car_model(calibration.load(args.calibration))
    layout = DEFAULT_LAYOUT
    print("car model: %s" % (car,))
    if args.solve:
        run_solver(layout, car)
        return 0
    ok = True
    for p in args.profiles or paths.PROFILE_KEYS:
        rep = check_profile(p, layout, car)
        print_report(rep, args.segments)
        ok = ok and rep.ok
        if args.png:
            try:
                out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots")
                os.makedirs(out, exist_ok=True)
                f = os.path.join(out, "%s.png" % p)
                plot_profile(p, layout, car, f)
                print("  saved %s" % f)
            except ImportError:
                print("  (matplotlib not installed: no PNG)")
    print("\n%s" % ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED: do not run these paths on the car"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
