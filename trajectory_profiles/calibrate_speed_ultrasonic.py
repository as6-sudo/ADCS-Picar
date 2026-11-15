#!/usr/bin/env python3
"""Hands-free speed_table calibration with the ultrasonic sensor.

Point the car straight at a flat wall about 1.5 m away and run:

    python3 trajectory_profiles/calibrate_speed_ultrasonic.py

For every speed command the car drives straight toward the wall for three
different durations. Before and after each drive it parks and takes a median
of static ultrasonic readings, so the distance travelled (start-up and coast
included) is measured without a tape measure. A least-squares fit of
d = v * (t - lag) over the three runs gives the same [cmd, cm/s, lag_s] rows
the manual speed_table check writes. After each run the car reverses back to
its starting distance using the same sensor.

Safety: every drive stops at once on two valid readings below --stop-cm
(default 40 cm), and no run is planned that would end closer than
--margin-cm (default 55 cm). Reverse moves are bounded by the sensor reading
and a timeout. Emergency stop: Ctrl+C.

Writes speed_table and checks.speed_table to calibration.json only if every
command gave a clean fit; --no-save prints the result without saving.
"""
import argparse
import math
import os
import statistics
import sys
import time
import warnings

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trajectory_profiles import calibration  # noqa: E402
from trajectory_profiles.motion import Motion  # noqa: E402

SPEED_CMDS = (1, 15, 30, 45, 60, 75, 90)   # same commands as calibrate.py
FRACTIONS = (0.4, 0.7, 1.0)                # run durations as fractions of the longest
MAX_RUN_S = 2.0
VALID_MIN_CM, VALID_MAX_CM = 2.0, 400.0
REVERSE_CMD = 30


def valid(d):
    try:
        d = float(d)
    except (TypeError, ValueError):
        return None
    return d if VALID_MIN_CM <= d <= VALID_MAX_CM else None


def static_distance(car, n=15, settle=0.6, max_tries=60):
    """Median of n valid readings with the car parked; None if the sensor fails."""
    time.sleep(settle)
    vals = []
    for _ in range(max_tries):
        d = valid(car.get_distance())
        if d is not None:
            vals.append(d)
            if len(vals) >= n:
                break
        time.sleep(0.03)
    if len(vals) < n // 2:
        return None, 0.0
    return statistics.median(vals), statistics.pstdev(vals)


def battery(car=None):
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            from robot_hat.utils import get_battery_voltage
            return round(float(get_battery_voltage()), 2)
    except Exception:
        return None


def drive(car, cmd, t_plan, stop_cm, clock=None):
    """Drive forward for t_plan seconds (or until the wall is too close).

    Returns (actual drive time, aborted, [(t, reading), ...])."""
    clock = clock or time.monotonic
    trace, low = [], 0
    car.set_dir_servo_angle(0)
    time.sleep(0.3)
    t0 = clock()
    car.forward(cmd)
    aborted = False
    while True:
        t = clock() - t0
        if t >= t_plan:
            break
        d = valid(car.get_distance())
        t = clock() - t0
        if d is not None:
            trace.append((round(t, 3), d))
            low = low + 1 if d < stop_cm else 0
            if low >= 2:
                aborted = True
                break
        # keep the loop tight near the end so the stop is on time
        if t_plan - t > 0.03:
            time.sleep(0.005)
    car.stop()
    return clock() - t0, aborted, trace


def reverse_to(car, target_cm, timeout_s=8.0, clock=None):
    """Back up until the wall reads target_cm. Returns the last valid reading."""
    clock = clock or time.monotonic
    car.set_dir_servo_angle(0)
    time.sleep(0.3)
    t0 = clock()
    car.backward(REVERSE_CMD)
    last, high = None, 0
    try:
        while clock() - t0 < timeout_s:
            d = valid(car.get_distance())
            if d is not None:
                last = d
                high = high + 1 if d >= target_cm else 0
                if high >= 2:
                    break
            time.sleep(0.01)
    finally:
        car.stop()
    return last


def fit(rows):
    """Least squares of D = v*t - v*lag over (t, D) rows -> (v, lag, rms residual)."""
    n = len(rows)
    mt = sum(t for t, _ in rows) / n
    md = sum(d for _, d in rows) / n
    stt = sum((t - mt) ** 2 for t, _ in rows)
    if stt <= 0:
        return None
    v = sum((t - mt) * (d - md) for t, d in rows) / stt
    if v <= 0:
        return None
    b = md - v * mt                      # D = v t + b, b = -v lag
    lag = -b / v
    rms = math.sqrt(sum((d - (v * t + b)) ** 2 for t, d in rows) / n)
    return v, lag, rms


def trace_speed(trace):
    """Cruise speed from the moving readings (cross-check), ignoring the first 0.3 s."""
    pts = [(t, d) for t, d in trace if t > 0.3]
    if len(pts) < 5:
        return None
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    md = sum(d for _, d in pts) / n
    stt = sum((t - mt) ** 2 for t, _ in pts)
    if stt <= 0:
        return None
    return -sum((t - mt) * (d - md) for t, d in pts) / stt


def calibrate(car, cal, *, stop_cm=40.0, margin_cm=55.0, min_start_cm=100.0, log=print,
              cmds=SPEED_CMDS):
    m = Motion(calibration.tables(cal))
    start, sd = static_distance(car)
    if start is None:
        log("Ultrasonic gives no valid readings: is the wall in front of the car?")
        return None
    log("Wall at %.1f cm (spread %.1f). Battery: %s V" % (start, sd, battery(car)))
    if start < min_start_cm:
        log("Too close: need at least %.0f cm to the wall. Move the car back." % min_start_cm)
        return None
    home = start
    room = home - margin_cm
    table, report = [], []
    for cmd in cmds:
        est = m.straight_speed(cmd)
        guess = est * 1.4                         # assume faster than the estimate until measured
        t_max = min(MAX_RUN_S, room / (1.15 * max(guess, 1.0)))
        runs, cruise = [], []
        for frac in FRACTIONS:
            t_plan = round(t_max * frac, 3)
            d0, s0 = static_distance(car)
            if d0 is None:
                log("  sensor lost the wall; stopping")
                return None
            if d0 - guess * t_plan * 1.15 < margin_cm:
                log("  cmd %d %.2fs: not enough room from %.1f cm; skipping this run" % (cmd, t_plan, d0))
                continue
            t_act, aborted, trace = drive(car, cmd, t_plan, stop_cm)
            d1, s1 = static_distance(car)
            if d1 is None:
                log("  sensor lost the wall after the run; stopping")
                return None
            moved = d0 - d1
            v_tr = trace_speed(trace)
            if v_tr and 1.0 < v_tr < 150.0:
                cruise.append(v_tr)
            else:
                v_tr = None
            log("  cmd %2d  t %.2fs  %6.1f -> %6.1f cm  moved %5.1f cm%s%s"
                % (cmd, t_act, d0, d1, moved,
                   "  (cruise %.1f cm/s)" % v_tr if v_tr else "",
                   "  ABORTED: wall closer than %.0f cm" % stop_cm if aborted else ""))
            if aborted:
                return None
            runs.append((t_act, moved))
            # re-plan the longer runs from what this run really did
            measured = v_tr if v_tr and 1.0 < v_tr < 150.0 else None
            if measured is None and t_act > 0.2 and moved > 0:
                measured = moved / t_act
            if measured and measured > 1.0:
                guess = measured
                t_max = min(MAX_RUN_S, room / (1.15 * guess))
            back = reverse_to(car, home)
            if back is not None and back < home - 5:
                back = reverse_to(car, home)        # a stray high reading stopped it early
            if back is None:
                log("  sensor lost the wall while reversing; stopping")
                return None
        f = fit(runs) if len(runs) >= 2 else None
        if f is None:
            log("  cmd %d: no usable fit (runs %s)" % (cmd, runs))
            report.append((cmd, None))
            continue
        v, lag, rms = f
        v_cr = statistics.median(cruise) if cruise else None
        flag = ""
        if rms > 1.5:
            flag += "  NOISY fit"
        if len(runs) < 3:
            flag += "  only %d runs" % len(runs)
        if v_cr and abs(v_cr - v) / v > 0.15:
            flag += "  cruise differs %.0f%%" % (100 * (v_cr - v) / v)
        log("  => cmd %2d: %.1f cm/s, lag %.3f s, fit rms %.2f cm (estimate was %.1f)%s"
            % (cmd, v, lag, rms, est, flag))
        table.append([cmd, round(v, 2), round(max(0.0, lag), 3)])
        report.append((cmd, (v, lag, rms, v_cr, len(runs))))
    log("Battery after: %s V" % battery(car))
    return table, report


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stop-cm", type=float, default=40.0, help="hard stop distance to the wall")
    ap.add_argument("--margin-cm", type=float, default=55.0, help="no run planned to end closer than this")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--cmds", type=lambda s: tuple(int(x) for x in s.split(",")), default=SPEED_CMDS,
                    help="comma list of speed commands to (re)measure; others keep their saved rows")
    args = ap.parse_args()
    from picarx import Picarx
    car = Picarx()
    cal = calibration.load()
    cal.setdefault("checks", {})
    try:
        res = calibrate(car, cal, stop_cm=args.stop_cm, margin_cm=args.margin_cm, cmds=args.cmds)
    except KeyboardInterrupt:
        print("\nInterrupted; nothing saved.")
        res = None
    finally:
        car.stop()
        car.set_dir_servo_angle(0)
    if res is None:
        print("speed_table NOT saved.")
        return 1
    table, report = res
    ok = len(table) == len(args.cmds) and all(r and r[2] <= 1.5 and r[4] >= 3 for _, r in report)
    if tuple(args.cmds) != SPEED_CMDS and cal.get("speed_table"):
        merged = {r[0]: r for r in cal["speed_table"]}
        merged.update({r[0]: r for r in table})
        table = [merged[c] for c in sorted(merged)]
        ok = ok and bool(cal.get("checks", {}).get("speed_table"))
    print("\nspeed_table:", table)
    if args.no_save:
        print("--no-save: calibration.json unchanged.")
        return 0 if ok else 1
    if len(table) >= 3:
        cal["speed_table"] = table
        cal["checks"]["speed_table"] = bool(ok)
        calibration.save(cal)
        print("Saved to %s (checks.speed_table = %s)." % (calibration.PATH, ok))
    else:
        print("Fewer than 3 commands fitted; not saved.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
