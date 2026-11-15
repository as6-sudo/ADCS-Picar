"""The segment executor: drives a list of segments open-loop, with sensor
checks, and keeps a command log that can be retraced (brief section 6).

Rules implemented here:
  1. Distance -> time from the calibrated speed table (per command, with a
     start/stop lag) and the arc-speed factor; radius -> steering from the
     calibrated radius table (motion.py).
  2. NO SPEED CHANGES IN THE MIDDLE OF AN ARC. Ramps are cut into the first
     and last straight of a run only. If a run ends on an arc it just stops
     at the end of it (the only allowed "deceleration" on an arc).
  3. Steering changes: stopped mode (LH) does stop -> steer -> wait for the
     swing + settle -> go. Rolling mode (HH/HL) issues each steering change
     half a swing BEFORE the segment boundary, so the curvature ramp is
     centred on the boundary and the heading change comes out as planned.
     picarx's forward()/backward() set the inner-wheel slowdown from the
     steering angle at call time, so the motor command is re-sent after
     every steering change.
  4. Every segment is logged with its ACTUAL driven time and the distance
     that time corresponds to; `retrace(log)` drives that backwards.
  5. Tape stop on the final straight (or the last N cm), TAPE_MISSED fallback.
  6. Forward moves stop on the hard minimum (2 readings < 12 cm). Reverse
     moves are unguarded (no rear sensor): keep the area behind A clear.
"""
import collections
import math
import threading
from dataclasses import dataclass, field

from obs_avoidance_profile_new.params import HARD_CONFIRM_READINGS, HARD_MIN_CM, LOOP_S
from obs_avoidance_profile_new.robot import EmergencyStop, EStop, ProfileAbort, RealClock, validate_distance

from . import geometry as g

NO_READING = 300.0          # picarx -1 = 10 timeouts in a row = nothing in range (as time_traversal.core)
__all__ = ["Executor", "EStop", "EmergencyStop", "ProfileAbort", "RealClock", "HardStop",
           "SensorStop", "TapeSpec", "LogEntry", "Ranger", "ThreadedRanger"]


class HardStop(ProfileAbort):
    """Something is closer than the hard minimum in front of the car."""


# ------------------------------------------------------------------ sensors
class Ranger:
    """Ultrasonic readings, polled in the control loop (simulator)."""

    def __init__(self, car, clock):
        self.car, self.clock = car, clock
        self.invalid = 0
        # A synchronous read blocks for this long (simulator: SimCar.READ_TIME).
        # The control loop skips a read when a deadline is closer than this.
        self.read_time = getattr(car, "READ_TIME", 0.0)

    def start(self):
        pass

    def stop(self):
        pass

    def _convert(self, raw):
        if raw == -1:
            return NO_READING
        d = validate_distance(raw)
        if d is None:
            self.invalid += 1
        return d

    def poll(self):
        """New readings since the last poll: list of validated cm (invalid ones dropped)."""
        d = self._convert(self.car.get_distance())
        return [] if d is None else [d]


class ThreadedRanger(Ranger):
    """Reads the ultrasonic in a background thread (real car).

    A picarx read can block for ~0.21 s when nothing echoes (10 x 20 ms
    timeouts), which would make segment timing late. The thread only touches
    the ultrasonic GPIO pins; motors, servos and the grayscale ADC stay on
    the main thread."""

    def __init__(self, car, clock):
        super().__init__(car, clock)
        self.read_time = 0.0                     # never blocks the control loop
        self._q = collections.deque(maxlen=100)
        self._run = threading.Event()
        self._thread = None

    def start(self):
        self._run.set()

        def loop():
            while self._run.is_set():
                try:
                    self._q.append(self.car.get_distance())
                except Exception:
                    self._q.append(None)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._run.clear()

    def poll(self):
        out = []
        while self._q:
            d = self._convert(self._q.popleft())
            if d is not None:
                out.append(d)
        return out


def is_dark(car, thresholds):
    try:
        vals = car.get_grayscale_data()
    except Exception:
        return False
    return any(v < t for v, t in zip(vals, thresholds))


# ------------------------------------------------------------------ data
@dataclass(frozen=True)
class TapeSpec:
    """Tape stop for the end of a run (from Layout.tape[target])."""
    mode: str = "final_straight"        # or "last_n"
    last_n_cm: float = 12.0
    arm_window_cm: float = 15.0
    miss_margin_cm: float = 4.0


@dataclass(frozen=True)
class SensorStop:
    """Stop the LAST segment when an obstacle is confirmed closer than threshold."""
    threshold_cm: float
    confirm: int = 3
    hit_event: str = "OBSTACLE_STOP"
    miss_event: str = "NO_OBSTACLE"


@dataclass
class LogEntry:
    leg: str
    index: int
    planned: g.Segment
    driven: g.Segment
    steer: float
    cmd: int
    planned_s: float
    actual_s: float
    event: str = ""


@dataclass
class _Piece:
    seg: int                # index of the segment in the run
    length: float           # cm of rear-axle path
    cmd: int
    steer: float
    v: float                # ground speed this command gives at this steering
    t0: float = 0.0
    t1: float = 0.0
    lag: float = 0.0        # start-up lag included at the front of this piece


@dataclass
class RunResult:
    entries: list = field(default_factory=list)
    events: list = field(default_factory=list)     # (name, detail)


# ------------------------------------------------------------------ executor
class Executor:
    def __init__(self, car, motion, clock=None, estop=None, ranger=None, record=None, log=print,
                 ramp_cm=8.0):
        self.car = car
        self.m = motion
        self.ramp_cm = ramp_cm          # CarModel.ramp_cm
        self.clock = clock or RealClock()
        self.estop = estop or EStop()
        self.ranger = ranger or Ranger(car, self.clock)
        self.record = record or (lambda **kw: None)
        self.log = log
        self.steer = 0.0
        self.pan = 0.0
        self.moving = False
        self.coasting = False       # stopped, but maybe still rolling to a halt

    # ---------------------------------------------------------- actuators
    def _set_steer(self, a):
        a = round(float(a), 2)
        if a != self.steer:
            self.car.set_dir_servo_angle(a)
            self.steer = a

    def _drive(self, direction, cmd):
        (self.car.forward if direction == g.FORWARD else self.car.backward)(int(cmd))
        self.moving = True

    def stop(self):
        self.car.stop()
        self.coasting = self.coasting or self.moving
        self.moving = False

    def wait(self, seconds):
        """Wait, checking the emergency stop. Any wait also lets the car come to rest."""
        if seconds >= self.m.t["stop_settle_s"]:
            self.coasting = False
        end = self.clock.now() + seconds
        while True:
            self.estop.check()
            left = end - self.clock.now()
            if left <= 1e-6:
                return
            self.clock.sleep(min(LOOP_S, left))

    def steer_stopped(self, a, settle=True):
        """Stopped: set the steering and wait for the servo to get there."""
        swing = self.m.swing_time(self.steer, a)
        self._set_steer(a)
        if swing > 0:
            self.wait(swing + (self.m.t["servo_settle_s"] if settle else 0.0))

    def pan_to(self, target, dps, limit=60.0):
        """Camera head to `target` degrees (+ = right) at `dps` degrees/second."""
        target = max(-limit, min(limit, target))
        step = max(1.0, dps * LOOP_S)
        while abs(target - self.pan) > 1e-6:
            self.estop.check()
            self.pan += max(-step, min(step, target - self.pan))
            self.car.set_cam_pan_angle(round(self.pan, 1))
            self.clock.sleep(LOOP_S)

    def centre(self):
        self.stop()
        self.car.set_dir_servo_angle(0)
        self.car.set_cam_pan_angle(0)
        self.car.set_cam_tilt_angle(0)
        self.steer, self.pan = 0.0, 0.0
        self.clock.sleep(self.m.t["servo_settle_s"] + self.m.swing_time(-30, 30))

    def safe_shutdown(self):
        """Used in `finally`: must never raise. Stops, then centres every servo."""
        for fn in (self.car.stop,
                   lambda: self.car.set_dir_servo_angle(0),
                   lambda: self.car.set_cam_pan_angle(0),
                   lambda: self.car.set_cam_tilt_angle(0),
                   self.car.stop,
                   self.ranger.stop):
            try:
                fn()
            except Exception:
                pass
        self.steer, self.pan, self.moving = 0.0, 0.0, False

    # ---------------------------------------------------------- planning
    def _pieces(self, run, speed, ramp_fracs):
        """Cut a same-direction run into constant-command pieces. Ramps go on
        the first/last straight only (never inside an arc)."""
        pieces = []
        n = len(run)
        ramp_cm = self.ramp_cm
        for i, s in enumerate(run):
            steer = self.m.steer_for(s.side, s.radius)
            parts = [(s.length, 1.0)]
            if ramp_fracs and not s.is_arc and s.length > 0:
                up = i == 0
                down = i == n - 1
                r = min(ramp_cm, s.length / (2 if (up and down) else 1))
                step = r / len(ramp_fracs)
                head = [(step, f) for f in ramp_fracs] if up else []
                tail = [(step, f) for f in reversed(ramp_fracs)] if down else []
                mid = s.length - step * (len(head) + len(tail))
                parts = head + ([(mid, 1.0)] if mid > 1e-9 else []) + tail
            for length, frac in parts:
                cmd = self.m.command_for(speed * frac, steer)
                v = self.m.ground_speed(cmd, steer)       # what this command really gives
                pieces.append(_Piece(i, length, cmd, steer, v))
        t = 0.0
        tau = self.m.t["speed_tau_s"]
        for k, p in enumerate(pieces):
            p.t0 = t
            if k == 0:
                # from rest: dead time + the speed response. The response comes back
                # as the coast after stop(), which is why the stop is issued tau
                # early (see _run_one); start + stop together = the calibrated lag.
                p.lag = self.m.lag(p.cmd) + tau
            else:
                # a speed change mid-run: the car takes ~tau to follow, so its position
                # lags by dv * tau; give this piece the time to make that up
                p.lag = tau * (p.v - pieces[k - 1].v) / p.v
            t += p.lag + p.length / p.v
            p.t1 = t
        return pieces

    # ---------------------------------------------------------- run
    def run(self, segments, *, leg, speed, rolling=True, ramp_fracs=(), tape=None,
            sensor_stop=None, on_segment_done=None, guard=True):
        """Drive `segments`. Returns a RunResult with one LogEntry per segment.

        rolling: HH/HL (steer while moving, stop only on direction changes);
                 False: LH (stop -> steer -> settle -> go for every segment).
        tape:    TapeSpec for the end of the LAST run, or None.
        sensor_stop: SensorStop for the last segment, or None.
        on_segment_done(i): called when segment i (index in `segments`) ends.
        """
        res = RunResult()
        segments = [s for s in segments if s.length > 1e-6]
        # group into runs that can be driven without stopping
        runs, cur = [], []
        for s in segments:
            if cur and (not rolling or s.direction != cur[-1].direction):
                runs.append(cur)
                cur = []
            cur.append(s)
        if cur:
            runs.append(cur)
        base = 0
        for k, run in enumerate(runs):
            last = k == len(runs) - 1
            self._run_one(run, base, leg, speed, ramp_fracs if rolling else (), res,
                          tape if last else None, sensor_stop if last else None, on_segment_done, guard)
            base += len(run)
            if res.events and res.events[-1][0] in ("OBSTACLE_STOP", "NO_OBSTACLE"):
                break
        return res

    def retrace(self, entries, **kw):
        """Drive a command log backwards: reverse order, F<->B, same steering,
        the logged (actual) distances through the same timing model."""
        return self.run(g.retrace([e.driven for e in entries]), **kw)

    def _run_one(self, run, base, leg, speed, ramp_fracs, res, tape, sensor_stop, done_cb, guard):
        pieces = self._pieces(run, speed, ramp_fracs)
        direction = run[0].direction
        if self.coasting:
            self.wait(self.m.t["stop_settle_s"])     # come to rest before steering / reversing
            self.coasting = False
        forward = direction == g.FORWARD
        tau = self.m.t["speed_tau_s"]
        end_t = pieces[-1].t1 - tau          # the car coasts ~v*tau after stop()

        # steering timeline: first steering is set while stopped; later changes
        # are issued half a swing before the boundary (rolling) - see rule 3.
        self.steer_stopped(pieces[0].steer)
        timeline = []          # (t, kind, value)
        for t, a, cmd in self._steer_changes(pieces):
            timeline.append((t, "steer", (a, cmd)))
        for a, b in zip(pieces, pieces[1:]):
            if b.cmd != a.cmd or b.steer != a.steer:
                timeline.append((b.t0, "cmd", b.cmd))
        timeline.sort(key=lambda e: e[0])

        # tape arming window and cut-off
        last_piece = pieces[-1]
        arm_t, max_t = None, end_t
        if tape is not None:
            v_end = last_piece.v
            if tape.mode == "last_n":
                arm_t = end_t - tape.last_n_cm / v_end
            elif not run[-1].is_arc:
                seg_start = min(p.t0 for p in pieces if p.seg == len(run) - 1)
                arm_t = max(seg_start, end_t - self._time_back(pieces, tape.arm_window_cm))
            max_t = end_t + tape.miss_margin_cm / v_end
        sensor_t = None
        if sensor_stop is not None:
            sensor_t = min(p.t0 for p in pieces if p.seg == len(run) - 1)

        # go
        self.ranger.poll()                       # drop readings taken before this run
        t0 = self.clock.now()
        self._drive(direction, pieces[0].cmd)
        self.record(kind="start", leg=leg, direction=direction, cmd=pieces[0].cmd, steer=pieces[0].steer)
        hard = close = 0
        event, ti, seg_done = "", 0, 0
        try:
            while True:
                self.estop.check()
                t = self.clock.now() - t0
                while ti < len(timeline) and timeline[ti][0] <= t:
                    _, kind, val = timeline[ti]
                    if kind == "steer":
                        # picarx applies the inner-wheel slowdown for the new angle as soon
                        # as forward()/backward() is called, so switch to the next block's
                        # command together with the steering (the speed change is centred too)
                        self._set_steer(val[0])
                        self._drive(direction, val[1])
                    else:
                        self._drive(direction, val)
                    ti += 1
                while done_cb and seg_done < len(run) - 1 and t >= max(p.t1 for p in pieces if p.seg == seg_done):
                    done_cb(base + seg_done)
                    seg_done += 1
                due = min([max_t - t] + [e[0] - t for e in timeline[ti:ti + 1]])
                sensing = (forward and guard) or sensor_t is not None
                readings = self.ranger.poll() if sensing and due > self.ranger.read_time else []
                for d in readings:
                    if forward and guard:
                        hard = hard + 1 if d < HARD_MIN_CM else 0
                    if sensor_t is not None and t >= sensor_t:
                        # invalid readings were already dropped: they neither count nor reset
                        close = close + 1 if d < sensor_stop.threshold_cm else 0
                if hard >= HARD_CONFIRM_READINGS:
                    event = "HARD_STOP"
                    break
                if sensor_t is not None and close >= sensor_stop.confirm:
                    event = sensor_stop.hit_event
                    break
                if arm_t is not None and t >= arm_t and is_dark(self.car, self.m.t["grayscale_threshold"]):
                    event = "TAPE_STOP"
                    break
                if t >= max_t:
                    if tape is not None:
                        event = "TAPE_MISSED"
                    elif sensor_stop is not None:
                        event = sensor_stop.miss_event
                    break
                t = self.clock.now() - t0               # a read may have taken time
                nxt = min([max_t - t] + [e[0] - t for e in timeline[ti:ti + 1]])
                self.clock.sleep(max(1e-4, min(LOOP_S, nxt)))
        finally:
            self.stop()
        actual = self.clock.now() - t0

        # log each segment that was (at least partly) driven, with its actual
        # time and the distance that time corresponds to in the timing model
        for i, s in enumerate(run):
            ps = [p for p in pieces if p.seg == i]
            s_t0, planned_t1 = ps[0].t0, ps[-1].t1
            if s_t0 >= actual and i > 0:
                break
            last = i == len(run) - 1
            s_t1 = max(planned_t1, actual) if last else planned_t1   # tape fallback may drive past the plan
            stop_here = actual <= planned_t1 + 1e-9 or last
            dist = 0.0
            for p in ps:
                b = s_t1 if p is ps[-1] else p.t1
                dist += max(0.0, min(b, actual) - p.t0 - p.lag) * p.v
            if stop_here:
                # the coast after stop(): ~v*tau, or less if the car had only just started
                at = next((p for p in ps if p.t0 <= actual < p.t1), ps[-1])
                moving = actual - at.t0 - (at.lag - tau if at is pieces[0] else 0.0)
                dist += at.v * min(tau, max(0.0, moving))
            driven = s.with_amount(math.degrees(dist / s.radius) if s.is_arc else dist)
            real = max(0.0, min(s_t1, actual) - s_t0)
            e = LogEntry(leg, base + i, s, driven, ps[0].steer, ps[0].cmd, planned_t1 - s_t0, real,
                         event if stop_here else "")
            res.entries.append(e)
            self.record(kind="segment", leg=leg, index=e.index, segment=str(s), driven=str(driven),
                        steer=e.steer, cmd=e.cmd, planned_s=round(e.planned_s, 3), actual_s=round(real, 3),
                        event=e.event)
            if done_cb and event != "HARD_STOP" and seg_done <= i:
                done_cb(base + i)
                seg_done = i + 1
            if stop_here:
                break
        if event:
            res.events.append((event, "%.1f cm into the last segment" % res.entries[-1].driven.length))
            self.record(kind="event", leg=leg, event=event)
            self.log("    %s: %s" % (leg, event))
        if event == "HARD_STOP":
            raise HardStop("obstacle closer than %.0f cm in front of the car" % HARD_MIN_CM)

    def _steer_changes(self, pieces):
        """(time, angle, speed command) steering commands for a rolling run.

        Each change is centred on its boundary (issued half a swing early).
        A piece too short for the servo to settle on its own angle (e.g. the
        0.6 cm straight between MAIN's arcs) is treated as part of the
        transition: one direct swing, centred on that piece's middle."""
        # collapse runs of equal steering into blocks (start, end, angle)
        blocks = []             # [start, end, angle, first command]
        for p in pieces:
            if blocks and blocks[-1][2] == p.steer:
                blocks[-1][1] = p.t1
            else:
                blocks.append([p.t0, p.t1, p.steer, p.cmd])
        changed = True
        while changed:
            changed = False
            for k in range(1, len(blocks) - 1):
                prev, cur, nxt = blocks[k - 1], blocks[k], blocks[k + 1]
                need = (self.m.swing_time(prev[2], cur[2]) + self.m.swing_time(cur[2], nxt[2])) / 2
                if cur[1] - cur[0] < need:
                    mid = (cur[0] + cur[1]) / 2
                    prev[1], nxt[0] = mid, mid     # split the sliver between its neighbours
                    del blocks[k]
                    changed = True
                    break
            for k in range(1, len(blocks)):        # merge neighbours that now share an angle
                if blocks[k][2] == blocks[k - 1][2]:
                    blocks[k - 1][1] = blocks[k][1]
                    del blocks[k]
                    changed = True
                    break
        out = []
        for a, b in zip(blocks, blocks[1:]):
            half = self.m.swing_time(a[2], b[2]) / 2
            out.append((max(a[0], b[0] - half), b[2], b[3]))
        return out

    @staticmethod
    def _time_back(pieces, cm):
        """Time it takes to cover the last `cm` of the run."""
        t, left = 0.0, cm
        for p in reversed(pieces):
            take = min(left, p.length)
            t += take / p.v
            left -= take
            if left <= 1e-9:
                break
        return t
