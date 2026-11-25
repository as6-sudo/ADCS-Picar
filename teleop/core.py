"""Teleop control core: keyboard state in, motor/servo commands out.

Hardware-free and clock-driven, so the same code runs on the car
(server.py), in the laptop simulator (client.py --sim) and in the tests.
Call `keys()` / `command()` when input arrives and `tick()` every ~20 ms.

Safety, in every profile:
  * deadman: driving only while W / S is held; if no input arrives for
    WATCHDOG_S (laptop crashed, Wi-Fi dropped, window lost focus) the car
    stops and every macro is cancelled;
  * Space stops at once (no late stop, no ramp) and cancels any macro;
  * forward motion stops on 2 readings < 12 cm (the repo's hard minimum)
    and stays blocked until W is released. Reverse is UNGUARDED (no rear
    sensor): keep the area behind the car clear.
"""
import math
import random

from obs_avoidance_profile_new.params import HARD_CONFIRM_READINGS, HARD_MIN_CM
from trajectory_profiles import paths

from .styles import INSTANT, ORDER, STYLES

WATCHDOG_S = 0.5
DRIVE_KEYS = ("w", "a", "s", "d", "up", "down", "left", "right")
PAN_LIMIT, TILT_MIN, TILT_MAX = 60.0, -30.0, 30.0
SCALE_MIN, SCALE_MAX, SCALE_STEP = 0.5, 1.5, 0.1
MAX_STEER = 30.0
OBSTACLE_SIDE = +1          # the storyboard obstacle slides in on the car's right: look right (+ = right)


class TeleopCore:
    def __init__(self, car, motion, clock, ranger, car_model, style="HH", record=None, seed=None):
        self.car, self.m, self.clock, self.ranger, self.cm = car, motion, clock, ranger, car_model
        self.style = STYLES[style]
        self.record = record or (lambda kind, **kw: None)
        self.rng = random.Random(seed)
        now = clock.now()
        self.t_last = self.last_input = now
        self.dt = 0.0
        self.held = frozenset()
        self.prev_held = frozenset()
        self.v = 0.0                    # signed planned ground speed, cm/s
        self.steer = 0.0                # slewed steering target, deg (+ = right)
        self.out = (0, 0, 0.0)          # last (direction, command, steering) sent
        self.pan = self.tilt = 0.0      # head as the operator set it
        self.pan_out = self.tilt_out = 0.0
        self.drift = 0.0                # LL head drift
        self.jitter = 0.0               # LL speed jitter
        self.wob_phase = 0.0
        self.scale = 1.0
        self.pause_until = 0.0
        self.late_until = None
        self.late_v = 0.0
        self.blocked = False            # forward blocked after a hard stop, until W is released
        self.hard = 0
        self.dist = None
        self.macro = None
        self.macro_name = ""
        self.rec = []                   # [[direction, command, steering, seconds], ...] since the mark
        self.retracing = False
        self.link_ok = True
        self.quit = False
        self.msg = "ready: %s" % self.style.name
        self.vmin = motion.ground_speed(1, 0.0)
        self.vmax = motion.max_speed(0.0)

    # ================================================================ input
    def keys(self, held):
        self.last_input = self.clock.now()
        self.held = frozenset(k for k in held if k in DRIVE_KEYS)
        if not self.link_ok:
            self.link_ok = True
            self._say("link back")

    def command(self, name, arg=None):
        self.last_input = self.clock.now()
        busy = self.macro is not None
        stopped = self.out[0] == 0 and self.v == 0.0 and not busy
        if name == "stop":
            self._halt("STOP")
        elif name == "quit":
            self._halt("quit")
            self.quit = True
        elif name == "profile":
            if arg not in STYLES:
                self._say("unknown profile %r" % arg)
            elif not stopped:
                self._say("stop first to change profile")
            else:
                self.style, self.scale = STYLES[arg], 1.0
                self.drift = self.jitter = 0.0
                self._say("profile %s" % self.style.name)
                self.record("event", detail="profile %s" % arg)
        elif name in ("faster", "slower"):
            d = SCALE_STEP if name == "faster" else -SCALE_STEP
            self.scale = round(min(SCALE_MAX, max(SCALE_MIN, self.scale + d)), 2)
            self._say("speed x%.1f = %.0f cm/s" % (self.scale, self.cruise()))
        elif name == "mark":
            self.rec = []
            self._say("mark set: R drives back to here")
            self.record("event", detail="mark")
        elif name == "centre":
            if busy:
                self._say("busy: %s" % self.macro_name)
            else:
                self.pan = self.tilt = self.drift = 0.0
                if stopped:
                    self.steer = 0.0
                self._say("centred")
        elif busy:
            self._say("busy: %s (Space cancels)" % self.macro_name)
        elif name in ("glance_left", "glance_right"):
            self._start(name.replace("_", " "), self._glance(-1 if name == "glance_left" else +1))
        elif name == "wiggle":
            if not stopped:
                self._say("stop first to wiggle")
            else:
                self._start("happy wiggle", self._wiggle())
        elif name == "surprise":
            self._start("surprise", self._surprise())
        elif name == "retrace":
            if not stopped:
                self._say("stop first to retrace")
            else:
                self._start("retrace", self._retrace())
        else:
            self._say("unknown command %r" % name)

    # ================================================================ loop
    def tick(self):
        now = self.clock.now()
        self.dt = max(0.0, min(0.1, now - self.t_last))
        self.t_last = now

        forward = self.out[0] > 0
        for d in self.ranger.poll():
            self.dist = d
            self.hard = self.hard + 1 if (forward and d < HARD_MIN_CM) else 0
        if not forward:
            self.hard = 0

        if now - self.last_input > WATCHDOG_S and self.link_ok:
            self.link_ok = False
            self.held = frozenset()
            self._halt("NO INPUT for %.1f s: stopped" % WATCHDOG_S)

        edges = self.held - self.prev_held
        self.prev_held = self.held

        if forward and self.hard >= HARD_CONFIRM_READINGS:
            self.hard = 0
            self.blocked = "w" in self.held or self.macro is not None
            self._halt("HARD STOP: something < %.0f cm ahead" % HARD_MIN_CM)
        if self.blocked and "w" not in self.held:
            self.blocked = False

        if self.macro is not None:
            try:
                next(self.macro)
            except StopIteration:
                self.macro, self.macro_name = None, ""
        else:
            self._manual(edges)
        self._head()
        self._record_tick()

    # ================================================================ manual driving
    def cruise(self):
        return min(self.vmax, max(self.vmin, self.style.cruise_cm_s * self.scale))

    def _manual(self, edges):
        st, now = self.style, self.clock.now()
        if st.pivots and edges & {"a", "d"}:
            if self.v != 0.0 or self.out[0] != 0:
                self._say("LH: stop before a pivot")
            elif now < self.pause_until:
                self._say("LH: pausing")
            else:
                right = "d" in edges
                r = self.cm.min_radius
                self._start("pivot %s" % ("right" if right else "left"),
                            self._segments(paths.right_pivot(r) if right else paths.left_pivot(r), self.cruise()))
                return
        fwd = "w" in self.held and not self.blocked
        back = "s" in self.held
        target = 0.0
        if fwd != back:
            target = self.cruise() * (1.0 if fwd else -st.reverse_frac)
        if now < self.pause_until:
            target = 0.0
        # LL: keeps rolling after the key is released
        if st.late_stop_s > 0:
            if target != 0.0:
                self.late_until = None
            elif self.v != 0.0 and self.late_until is None:
                self.late_until, self.late_v = now + st.late_stop_s, self.v
                self._say("...late stop")
            if self.late_until is not None and target == 0.0:
                if now < self.late_until:
                    target = self.late_v
                else:
                    self.late_until = -1.0          # expired: now really stop
        was_moving = self.v != 0.0
        if st.accel_cm_s2 == INSTANT:
            self.v = target
        else:
            step = st.accel_cm_s2 * self.dt
            self.v += max(-step, min(step, target - self.v))
        if was_moving and self.v == 0.0:
            self.late_until = None
            if st.pause_after_stop_s > 0:
                self.pause_until = now + st.pause_after_stop_s
        # steering
        want = 0.0
        if not st.pivots:
            want = st.steer_max * ((1 if "d" in self.held else 0) - (1 if "a" in self.held else 0))
        if st.steer_dps == INSTANT:
            self.steer = want
        else:
            step = st.steer_dps * self.dt
            self.steer += max(-step, min(step, want - self.steer))
        v, steer = self.v, self.steer
        if st.wobble_deg and v != 0.0:
            self.wob_phase += 2 * math.pi * st.wobble_hz * self.dt
            steer += st.wobble_deg * math.sin(self.wob_phase) + self.rng.gauss(0, st.wobble_deg * 0.15)
        if st.speed_jitter and v != 0.0:
            self.jitter += self.rng.gauss(0, 0.6) * self.dt
            self.jitter = max(-st.speed_jitter, min(st.speed_jitter, self.jitter))
            v *= 1.0 + self.jitter
        self._apply(v, steer)

    def _head(self):
        st = self.style
        if self.macro is None:
            step = st.head_dps * self.dt
            if "left" in self.held:
                self.pan -= step
            if "right" in self.held:
                self.pan += step
            if "up" in self.held:
                self.tilt += step
            if "down" in self.held:
                self.tilt -= step
            self.pan = max(-PAN_LIMIT, min(PAN_LIMIT, self.pan))
            self.tilt = max(TILT_MIN, min(TILT_MAX, self.tilt))
            if st.head_drift_deg:
                self.drift += self.rng.gauss(0, st.head_drift_deg * 0.8) * self.dt - self.drift * 0.2 * self.dt
                self.drift = max(-st.head_drift_deg, min(st.head_drift_deg, self.drift))
        pan = max(-PAN_LIMIT, min(PAN_LIMIT, self.pan + (self.drift if self.macro is None else 0.0)))
        if abs(pan - self.pan_out) >= 0.5:
            self.pan_out = round(pan, 1)
            self.car.set_cam_pan_angle(self.pan_out)
        if abs(self.tilt - self.tilt_out) >= 0.5:
            self.tilt_out = round(self.tilt, 1)
            self.car.set_cam_tilt_angle(self.tilt_out)

    # ================================================================ outputs
    def _apply(self, v, steer):
        """Ground speed (signed cm/s) + steering -> picarx command."""
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        if abs(v) < 0.5:
            self._send(0, 0, steer)
        else:
            self._send(1 if v > 0 else -1, self.m.command_for(abs(v), steer), steer)

    def _send(self, direction, cmd, steer):
        d0, c0, s0 = self.out
        steer = round(steer, 1)
        steer_changed = abs(steer - s0) >= 0.5 or (steer == 0.0 and s0 != 0.0)
        if steer_changed:
            self.car.set_dir_servo_angle(steer)
        else:
            steer = s0
        if direction == 0:
            if d0 != 0:
                self.car.stop()
        elif direction != d0 or cmd != c0 or steer_changed:
            # picarx sets the inner-wheel slowdown from the steering at call time:
            # re-send the motor command after every steering change
            (self.car.forward if direction > 0 else self.car.backward)(int(cmd))
        if direction == 0:
            cmd = 0
        if (direction, cmd) != (d0, c0):
            self.record("drive", direction=direction, cmd=cmd, steer=steer)
        self.out = (direction, cmd, steer)

    def _halt(self, why):
        if self.macro is not None:
            self.macro.close()
            self.macro, self.macro_name = None, ""
        self.v = 0.0
        self.late_until = None
        self._send(0, 0, self.out[2])
        self._say(why)
        self.record("event", detail=why)

    def _say(self, text):
        self.msg = text

    def _record_tick(self):
        d, c, s = self.out
        if self.retracing or (d == 0 and not self.rec):
            return
        if self.rec and self.rec[-1][:3] == [d, c, s]:
            self.rec[-1][3] += self.dt
        else:
            self.rec.append([d, c, s, self.dt])

    # ================================================================ macros
    def _start(self, name, gen):
        self.macro, self.macro_name = gen, name
        self._say(name)
        self.record("event", detail="macro %s" % name)

    def _wait(self, s):
        end = self.clock.now() + s
        while self.clock.now() < end - 1e-9:
            yield

    def _hold(self, direction, cmd, steer, s):
        end = self.clock.now() + s
        while self.clock.now() < end - 1e-9:
            self._send(direction, cmd, steer)
            yield

    def _stop_and_settle(self):
        moving = self.out[0] != 0
        self.v = 0.0
        self._send(0, 0, self.out[2])
        if moving:
            yield from self._wait(self.m.t["stop_settle_s"])

    def _segments(self, segs, cm_s):
        """Stop -> steer -> settle -> go for each segment (the LH way)."""
        for seg in segs:
            steer = self.m.steer_for(seg.side, seg.radius)
            if abs(steer - self.out[2]) >= 0.5:
                yield from self._stop_and_settle()
                swing = self.m.swing_time(self.out[2], steer)
                self._send(0, 0, steer)
                yield from self._wait(swing + self.m.t["servo_settle_s"])
            cmd = self.m.command_for(cm_s, steer)
            t = seg.length / self.m.ground_speed(cmd, steer) + self.m.lag(cmd)
            yield from self._hold(1 if seg.direction == "F" else -1, cmd, steer, t)
        yield from self._stop_and_settle()

    def _pan_to(self, target, dps):
        target = max(-PAN_LIMIT, min(PAN_LIMIT, target))
        while abs(target - self.pan) > 1e-6:
            step = max(0.5, dps * self.dt)
            self.pan += max(-step, min(step, target - self.pan))
            yield

    def _glance(self, side):
        st, back = self.style, self.pan
        yield from self._pan_to(side * st.glance_deg, st.head_dps)
        yield from self._wait(st.glance_hold_s)
        yield from self._pan_to(back, st.head_dps)

    def _wiggle(self):
        st = self.style
        yield from self._stop_and_settle()
        pan0 = self.pan
        for _ in range(st.wiggle_count):
            for s in (+1, -1):
                amp = st.wiggle_deg * (1.0 + (self.rng.uniform(-0.4, 0.4) if st.key == "LL" else 0.0))
                self._send(0, 0, s * amp)
                self.pan = pan0 - s * amp
                yield from self._wait(st.wiggle_period_s / 2)
        self._send(0, 0, 0.0)
        self.steer, self.pan = 0.0, pan0
        yield from self._wait(st.wiggle_period_s)

    def _surprise(self):
        st = self.style
        if st.surprise == "late_retrace" and self.out[0] != 0:
            d, c, s = self.out          # LL: doesn't react for a while
            yield from self._hold(d, c, s, st.late_stop_s)
        yield from self._stop_and_settle()
        self.late_until = None
        yield from self._wait(st.surprise_pause_s)
        if st.surprise == "backup":
            yield from self._segments(paths.EVENT_BACKUP, self.cruise())
            if st.look_after_surprise:
                yield from self._pan_to(OBSTACLE_SIDE * 30.0, st.head_dps)
                yield from self._wait(st.glance_hold_s)
                yield from self._pan_to(0.0, st.head_dps)
            yield from self._wait(st.surprise_pause_s)
            self.steer = self.out[2]
            self._say("surprise done: drive the new arc")
        else:
            yield from self._retrace()

    def _retrace(self):
        rec = [r for r in self.rec]
        while rec and rec[-1][0] == 0:
            rec.pop()
        if not any(r[0] for r in rec):
            self._say("nothing recorded since the mark (M)")
            return
        self.retracing = True
        try:
            yield from self._stop_and_settle()
            wob = self.style.wobble_deg
            phase = self.rng.uniform(0, 2 * math.pi)        # a different wobble on the way back
            for d, c, s, t in reversed(rec):
                if d == 0:
                    yield from self._hold(0, 0, s, t)
                    continue
                end = self.clock.now() + t
                while self.clock.now() < end - 1e-9:
                    extra = 0.0
                    if wob:
                        phase += 2 * math.pi * self.style.wobble_hz * self.dt
                        extra = 0.6 * wob * math.sin(phase)
                    self._send(-d, c, max(-MAX_STEER, min(MAX_STEER, s + extra)))
                    yield
            yield from self._stop_and_settle()
        finally:
            self.retracing = False
        self.rec = []
        self.steer = self.out[2]
        self._say("retrace done: back at the mark")

    # ================================================================ status
    def status(self):
        d, c, s = self.out
        st = self.style
        now = self.clock.now()
        if not self.link_ok:
            state = "NO INPUT"
        elif self.macro is not None:
            state = self.macro_name
        elif self.blocked:
            state = "BLOCKED (release W)"
        elif now < self.pause_until:
            state = "pause %.1f s" % (self.pause_until - now)
        elif self.late_until is not None and self.late_until > 0:
            state = "late stop"
        else:
            state = "driving" if d else "stopped"
        return {
            "profile": st.key, "profile_name": st.name, "scale": self.scale, "cruise": round(self.cruise(), 1),
            "dir": d, "cmd": c, "steer": s,
            "speed": round(d * self.m.ground_speed(c, s), 1) if d else 0.0,
            "pan": self.pan_out, "tilt": self.tilt_out,
            "dist": None if self.dist is None else round(self.dist, 1),
            "state": state, "msg": self.msg, "rec_s": round(sum(r[3] for r in self.rec if r[0]), 1),
            "profiles": list(ORDER),
        }

    def safe_shutdown(self):
        for fn in (self.car.stop, lambda: self.car.set_dir_servo_angle(0),
                   lambda: self.car.set_cam_pan_angle(0), lambda: self.car.set_cam_tilt_angle(0),
                   self.car.stop, self.ranger.stop):
            try:
                fn()
            except Exception:
                pass
