#!/usr/bin/env python3
"""Car side of the teleop link. Started by client.py over SSH; reads one
JSON message per line on stdin and writes status lines on stdout:

    in : {"t": "keys", "held": ["w", "a"]}          (~20 Hz, also the heartbeat)
         {"t": "cmd", "name": "profile", "arg": "HL"}
    out: {"t": "status", ...}                       (~10 Hz)
         {"t": "log", "msg": "..."}

The car stops if input stops for 0.5 s, and stops + centres every servo when
stdin closes (SSH dropped), on SIGTERM/SIGHUP, or on any error. A small CSV
of drive commands and events goes to teleop/logs/ (~1 KB per minute).

Manual start on the Pi (for debugging; normally client.py does this):
    ROBOT_HAT_GPIOCHIP=0 python3 -u -m teleop.server
"""
import argparse
import csv
import datetime
import json
import os
import queue
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from obs_avoidance_profile_new.params import LOOP_S  # noqa: E402
from trajectory_profiles import calibration  # noqa: E402
from trajectory_profiles.motion import Motion  # noqa: E402

from teleop.core import TeleopCore  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STATUS_EVERY = 5            # ticks (~10 Hz)


def emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


class CsvRecord:
    FIELDS = ["t", "kind", "direction", "cmd", "steer", "detail"]

    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "w", newline="")
        self.w = csv.DictWriter(self.f, self.FIELDS, extrasaction="ignore")
        self.w.writeheader()
        self.t0 = time.monotonic()
        self.path = path

    def __call__(self, kind, **kw):
        self.w.writerow(dict(kw, kind=kind, t="%.3f" % (time.monotonic() - self.t0)))
        self.f.flush()

    def close(self):
        self.f.close()


def make_car(sim):
    if sim:
        from trajectory_profiles.layout import DEFAULT as LAYOUT
        from trajectory_profiles.sim import ArenaSim
        from trajectory_profiles.executor import Ranger
        res = calibration.Resolved(None)
        s = ArenaSim(LAYOUT, res.car, tables=res.tables)
        return s, Ranger(s, s), res
    from picarx import Picarx
    from trajectory_profiles.executor import ThreadedRanger
    from obs_avoidance_profile_new.robot import RealClock
    res = calibration.Resolved(calibration.load())
    car = Picarx()
    return car, ThreadedRanger(car, RealClock()), res


class WallClock:
    def now(self):
        return time.monotonic()


class SimWallClock:
    """Real time, with the simulator stepped to match (server --sim)."""

    def __init__(self, sim):
        self.sim, self.t0 = sim, time.monotonic()

    def now(self):
        target = time.monotonic() - self.t0
        if target > self.sim.t:
            self.sim.sleep(target - self.sim.t)
        return self.sim.t


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated car (no hardware), for testing the link")
    ap.add_argument("--profile", default="HH")
    args = ap.parse_args(argv)

    stop_flag = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, lambda *_: stop_flag.set())

    car, ranger, res = make_car(args.sim)
    clock = SimWallClock(car) if args.sim else WallClock()
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    rec = CsvRecord(os.path.join(HERE, "logs", "teleop_%s%s.csv" % (stamp, "_sim" if args.sim else "")))
    core = TeleopCore(car, Motion(res.tables), clock, ranger, res.car, style=args.profile, record=rec)

    inbox = queue.Queue()

    def reader():
        for line in sys.stdin:
            line = line.strip()
            if line:
                inbox.put(line)
        inbox.put(None)             # EOF: the laptop went away
    threading.Thread(target=reader, daemon=True).start()

    emit({"t": "log", "msg": "teleop server up (%s), speed %.0f-%.0f cm/s, log %s"
          % ("SIM" if args.sim else "car", core.vmin, core.vmax, os.path.relpath(rec.path, os.path.dirname(HERE)))})
    if not args.sim:
        emit({"t": "log", "msg": "calibration: measured %s; the rest are defaults"
              % (", ".join(res.measured) or "nothing")})
    ranger.start()
    n = 0
    try:
        while not stop_flag.is_set() and not core.quit:
            t0 = time.monotonic()
            eof = False
            while True:
                try:
                    line = inbox.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    eof = True
                    break
                try:
                    m = json.loads(line)
                except ValueError:
                    continue
                if m.get("t") == "keys":
                    core.keys(m.get("held", []))
                elif m.get("t") == "cmd":
                    core.command(m.get("name"), m.get("arg"))
            if eof:
                core._halt("laptop disconnected")
                break
            core.tick()
            n += 1
            if n % STATUS_EVERY == 0:
                st = core.status()
                st["t"] = "status"
                if args.sim:
                    p = car.pose()
                    st["sim"] = {"x": round(p.x, 1), "y": round(p.y, 1), "h": round(p.h, 1),
                                 "obstacle": [round(v, 1) for v in car.obstacle]}
                emit(st)
            time.sleep(max(0.0, LOOP_S - (time.monotonic() - t0)))
    except Exception as e:          # never leave the car driving
        emit({"t": "log", "msg": "ERROR: %r" % e})
        raise
    finally:
        core.safe_shutdown()
        rec.close()
        try:
            emit({"t": "log", "msg": "teleop server stopped, car stopped and centred"})
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
