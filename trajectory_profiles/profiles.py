"""A whole take: every leg of a profile in one process, with the behaviour
(gaze, pauses, wiggle, operator cue) around the geometry from paths.py.

    take = Take("HH", executor, layout, car, motion)
    ok = take.run()            # or take.run(only_leg=3)

The car always ends stopped with every servo centred (safe_shutdown in
`finally`). LL is registered but raises NotImplementedError.
"""
import math
import statistics

from . import geometry as g, paths
from .executor import EmergencyStop, ProfileAbort, SensorStop, TapeSpec
from .params import PROFILES as BEHAVIOURS

PROFILE_KEYS = ("HH", "HL", "LH", "LL")


class Retake(ProfileAbort):
    """The take cannot be used (e.g. the obstacle was late): stop and end it."""


# ------------------------------------------------------------------ tape strips
def arrival_speed(beh, motion):
    """Ground speed at which a leg reaches its target (end of the ramp-down)."""
    frac = beh.ramp_fracs[0] if beh.ramp_fracs else 1.0
    cmd = motion.command_for(beh.cruise_cm_s * frac)
    return motion.ground_speed(cmd, 0.0)


def strip_x(target, direction, layout, car, motion, cm_s):
    """Where to tape the strip for `target` (x of its centre line).

    The grayscale module is `grayscale_offset` ahead of the car centre, and
    the car comes to rest `tape_overshoot` past the point where it saw the
    strip, so the strip goes that much early in the direction of travel."""
    t = layout.target(target)
    sign = 1.0 if direction == g.FORWARD else -1.0
    return t.x + car.grayscale_offset - sign * motion.tape_overshoot(cm_s)


def strips(profile, layout, car, motion):
    """{target: (x0, x1, y0, y1)} dark rectangles for every enabled tape stop
    of a profile. A is always reached in reverse, B/C/D forward."""
    beh = BEHAVIOURS[profile]
    v = arrival_speed(beh, motion)
    out = {}
    for name, tape in layout.tape.items():
        if not tape.enabled:
            continue
        x = strip_x(name, g.BACKWARD if name == "A" else g.FORWARD, layout, car, motion, v)
        y = layout.target(name).y
        w, l = layout.strip_width_cm / 2, layout.strip_length_cm / 2
        out[name] = (x - w, x + w, y - l, y + l)
    return out


def tape_spec(layout, target):
    t = layout.tape[target]
    if not t.enabled:
        return None
    return TapeSpec(t.mode, t.last_n_cm, t.arm_window_cm, t.miss_margin_cm)


# ------------------------------------------------------------------ the take
class Take:
    def __init__(self, profile, executor, layout, car, motion, speed_scale=1.0, sim=None,
                 log=print, cue_out=None, tape_stops=True):
        if profile == "LL":
            raise NotImplementedError("Profile LL is out of scope for this framework (brief section 1).")
        self.p = profile
        self.beh = BEHAVIOURS[profile]
        self.ex = executor
        self.layout, self.car, self.m = layout, car, motion
        self.speed = self.beh.cruise_cm_s * speed_scale
        self.sim = sim
        self.tape_stops = tape_stops     # False: no grayscale calibration -> timed stops
        self.log = log
        self.cue_out = cue_out or (lambda: print("\a\n" + "#" * 40 + "\n####" + " " * 12 + "PULL" + " " * 12 + "####\n"
                                                  + "#" * 40 + "\n", flush=True))
        self.pose = layout.target("A")       # where the car should be (plan)
        self.events = []                     # (leg, name, detail)
        self.cue_time = None

    # ---------------------------------------------------------- helpers
    def _event(self, leg, name, detail=""):
        self.events.append((leg, name, detail))
        self.ex.record(kind="event", leg=leg, event=name, detail=detail)
        self.log("  [%s] %s %s" % (leg, name, detail))

    def cue(self, leg):
        """Operator cue on the LAPTOP terminal (bell + big PULL). The robot
        itself never beeps: that would be on the video's soundtrack."""
        self.cue_time = self.ex.clock.now()
        self.cue_out()
        self._event(leg, "CUE", "PULL")
        if self.sim is not None:
            self.sim.start_pull(self.layout.operator_reaction_s)

    def drive(self, leg, segments, tape_target=None, **kw):
        kw.setdefault("speed", self.speed)
        kw.setdefault("rolling", self.beh.rolling)
        kw.setdefault("ramp_fracs", self.beh.ramp_fracs)
        res = self.ex.run(segments, leg=leg, tape=self.tape(tape_target), **kw)
        for name, detail in res.events:
            self.events.append((leg, name, detail))
        self.pose = g.end_pose(self.pose, [e.driven for e in res.entries], self.car.rear_to_centre)
        return res

    def tape(self, target):
        """TapeSpec for a target, or None (timed stop) if tape stops are off."""
        if not target or not self.tape_stops:
            return None
        return tape_spec(self.layout, target)

    def gaze_at(self, point, leg):
        """Turn the head toward a point (x, y), hold, and look forward again."""
        bearing = math.degrees(math.atan2(point[1] - self.pose.y, point[0] - self.pose.x))
        pan = -g.wrap180(bearing - self.pose.h)          # picarx: + pans RIGHT, headings are CCW
        self.ex.record(kind="gaze", leg=leg, pan=round(pan, 1))
        self.ex.pan_to(pan, self.beh.gaze_dps)
        self.ex.wait(self.beh.gaze_hold_s)
        self.ex.pan_to(0.0, self.beh.gaze_dps)

    def wiggle(self, leg):
        """Happy wiggle, standing still: steering and head swing a few times."""
        b = self.beh
        self.ex.record(kind="wiggle", leg=leg)
        for _ in range(b.wiggle_count):
            for s in (+1, -1):
                self.ex._set_steer(s * b.wiggle_deg)
                self.ex.car.set_cam_pan_angle(-s * b.wiggle_deg)
                self.ex.wait(b.wiggle_period_s / 2)
        self.ex._set_steer(0.0)
        self.ex.car.set_cam_pan_angle(0)
        self.ex.wait(b.wiggle_period_s)

    def confirm_obstacle(self, leg, window_s=0.3):
        """Validated ultrasonic readings for `window_s` at the stop; median."""
        self.ex.ranger.poll()
        vals, end = [], self.ex.clock.now() + window_s
        while self.ex.clock.now() < end:
            self.ex.estop.check()
            vals += self.ex.ranger.poll()
            self.ex.clock.sleep(0.02)
        return statistics.median(vals) if vals else None

    # ---------------------------------------------------------- legs
    def event_leg(self, leg):
        """HH/HL A->C: cue, drive the 'under' path a fixed distance, stop,
        confirm, surprise back-up, (look), re-plan to C (brief 5, 8)."""
        name = "leg %d" % leg.number
        self.cue(name)
        self.ex.wait(self.layout.cue[self.p])
        approach, backup, replan = (ph.segments for ph in leg.phases)
        # the approach starts on an arc: no ramp-up (constant speed from the start)
        self.drive(name, approach)
        d = self.confirm_obstacle(name)
        if d is None or d > self.layout.event_late_cm:
            # The re-plan to C crosses O0, where a late obstacle still is: never
            # drive it. The car is already stopped; end the take here.
            self._event(name, "OBSTACLE_LATE", "reading %s cm > %.0f"
                        % ("none" if d is None else "%.1f" % d, self.layout.event_late_cm))
            raise Retake("obstacle was late (not at O1 when the car stopped): RETAKE")
        else:
            self._event(name, "OBSTACLE_CONFIRMED", "%.1f cm" % d)
        self.ex.wait(self.beh.surprise_pause_s)
        self.drive(name, backup)
        if self.beh.look_at_obstacle:
            self.gaze_at(self.layout.o1, name)
        self.ex.wait(self.beh.surprise_pause_s)
        self.drive(name, replan, tape_target="C")

    def lh_obstacle_leg(self, leg):
        """LH A->C: lane, sensor stop on the lower lane (or the cap), then
        retrace the logged commands back to A."""
        name = "leg %d" % leg.number
        driven = leg.phases[0].segments
        lane, _ = paths.lh_lane_ac_planned(self.layout, self.car)
        to_lane = driven[:len(lane)]
        stop_seg = g.Segment("F", "0", 0, self.layout.lh_detect_cap_cm)   # the cap: never further
        n_pullout_pivot = 4                  # F 0 10 + the 3 steps of the first pivot

        def on_done(i):
            if i == n_pullout_pivot - 1:
                self.cue(name)

        res = self.drive(name, to_lane + [stop_seg],
                         sensor_stop=SensorStop(self.layout.lh_detect_cm, confirm=self.layout.lh_confirm_readings),
                         on_segment_done=on_done)
        self.ex.wait(self.beh.pause_at_target_s)
        back = self.ex.retrace(res.entries, leg=name, speed=self.speed, rolling=self.beh.rolling,
                               ramp_fracs=self.beh.ramp_fracs, tape=self.tape("A"))
        for n, detail in back.events:
            self.events.append((name, n, detail))
        self.pose = g.end_pose(self.pose, [e.driven for e in back.entries], self.car.rear_to_centre)

    def run(self, only_leg=None):
        """Run the take (or one leg). Returns True if it completed."""
        legs = paths.legs(self.p, self.layout, self.car)
        if only_leg is not None:
            legs = [l for l in legs if l.number == only_leg]
            if not legs:
                raise ValueError("profile %s has no leg %d" % (self.p, only_leg))
        ok = False
        try:
            self.ex.ranger.start()
            self.ex.centre()
            for leg in legs:
                name = "leg %d" % leg.number
                self.pose = self.layout.target(leg.start)
                self.log("[%s] %s: %s" % (self.p, name, leg.name))
                self.ex.record(kind="leg", leg=name, name=leg.name)
                if self.sim is not None and only_leg is not None:
                    self.sim.place_obstacle(self.layout.o0 if leg.phases[0].obstacle in ("O0", "SWEEP")
                                            else self.layout.o1)
                if leg.number in self.beh.gaze_before_legs:
                    t = self.layout.target(leg.end)
                    self.gaze_at((t.x, t.y), name)
                if leg.name.endswith("event"):
                    self.event_leg(leg)
                elif self.p == "LH" and leg.start == leg.end:
                    self.lh_obstacle_leg(leg)
                else:
                    self.drive(name, leg.segments, tape_target=leg.end)
                self.ex.wait(self.beh.pause_at_target_s)
            if self.beh.wiggle_at_end and (only_leg is None or only_leg == len(paths.legs(self.p, self.layout, self.car))):
                self.wiggle("end")
            ok = True
        except EmergencyStop as e:
            self.log("[%s] EMERGENCY STOP: %s" % (self.p, e))
            self._event("-", "ESTOP", str(e))
        except Retake as e:
            self.log("[%s] RETAKE: %s" % (self.p, e))
            self._event("-", "RETAKE", str(e))
        except ProfileAbort as e:
            self.log("[%s] aborted: %s" % (self.p, e))
            self._event("-", "ABORT", str(e))
        finally:
            self.ex.safe_shutdown()
        return ok
