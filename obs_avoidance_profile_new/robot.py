"""Blocking movement primitives on top of a Picarx (or SimCar).

Every primitive finishes its movement (and stops the motors) before it
returns, so a profile is just an ordered list of calls. Every wait goes
through `Robot.wait`, which checks the emergency stop every LOOP_S.

Only the Picarx methods already used by `time_traversal` are called:
forward, backward, stop, set_dir_servo_angle, set_cam_pan_angle,
set_cam_tilt_angle and get_distance.
"""
import sys
import threading
import time

from time_traversal.core import clamp, MAX_STEER, NO_READING

from .params import (CONFIRM_READINGS, HARD_CONFIRM_READINGS, HARD_MIN_CM,
                     LOOP_S, PAN_LIMIT_DEG, SERVO_SETTLE_S, VALID_MAX_CM,
                     VALID_MIN_CM)


class EmergencyStop(Exception):
    pass


class ProfileAbort(Exception):
    """A profile cannot continue safely (e.g. no obstacle found)."""


class RealClock:
    def now(self):
        return time.monotonic()

    def sleep(self, s):
        time.sleep(s)


class EStop:
    """Emergency stop flag. Triggered by Enter, SIGTERM, or code."""

    def __init__(self):
        self._event = threading.Event()
        self.reason = None

    def trigger(self, reason="emergency stop"):
        if not self._event.is_set():
            self.reason = reason
            self._event.set()

    @property
    def triggered(self):
        return self._event.is_set()

    def check(self):
        if self._event.is_set():
            raise EmergencyStop(self.reason)

    def listen_for_enter(self):
        """Background thread: pressing Enter in the terminal stops the car."""
        def watch():
            try:
                if sys.stdin.readline():
                    self.trigger("Enter pressed")
            except Exception:
                pass
        threading.Thread(target=watch, daemon=True).start()


def validate_distance(raw):
    """Return a plausible distance in cm, or None for an invalid reading."""
    try:
        d = float(raw)
    except (TypeError, ValueError):
        return None
    if d != d or d < VALID_MIN_CM or d > VALID_MAX_CM:   # NaN, -1, -2, 0, out of range
        return None
    return d


class Robot:
    def __init__(self, car, clock=None, estop=None, speed_scale=1.0, log=print):
        self.car = car
        self.clock = clock or RealClock()
        self.estop = estop or EStop()
        self.speed_scale = speed_scale
        self.log = log
        self.pan = 0.0
        self.steer = 0
        self.trace = []            # (t, kind, data) for logging and tests
        self.invalid_readings = 0

    # ------------------------------------------------------------ helpers
    def record(self, kind, **data):
        self.trace.append((self.clock.now(), kind, data))

    def wait(self, seconds):
        end = self.clock.now() + seconds
        while True:
            self.estop.check()
            left = end - self.clock.now()
            if left <= 1e-6:
                return
            self.clock.sleep(min(LOOP_S, left))

    def scaled(self, speed):
        return int(round(clamp(speed * self.speed_scale, 0, 100)))

    def read_distance(self):
        """Valid distance in cm; NO_READING (300) for -1, which picarx only
        returns after 10 echo timeouts in a row, i.e. nothing in range (same
        rule as time_traversal.core); None for other invalid readings, which
        are ignored by the callers."""
        raw = self.car.get_distance()
        if raw == -1:
            return NO_READING
        d = validate_distance(raw)
        if d is None:
            self.invalid_readings += 1
        return d

    # ------------------------------------------------------------ actuators
    def stop(self):
        self.car.stop()
        self.record("stop")

    def set_steer(self, angle):
        a = int(clamp(angle, -MAX_STEER, MAX_STEER))
        if a != self.steer:
            self.car.set_dir_servo_angle(a)
            self.steer = a
            self.record("steer", angle=a)

    def center(self):
        """Stop, then put steering and camera pan/tilt at neutral."""
        self.car.stop()
        self.car.set_dir_servo_angle(0)
        self.car.set_cam_pan_angle(0)
        self.car.set_cam_tilt_angle(0)
        self.steer, self.pan = 0, 0.0
        self.record("center")
        self.clock.sleep(SERVO_SETTLE_S)

    def safe_shutdown(self):
        """Used in `finally`: must never raise."""
        for fn in (self.car.stop,
                   lambda: self.car.set_dir_servo_angle(0),
                   lambda: self.car.set_cam_pan_angle(0),
                   lambda: self.car.set_cam_tilt_angle(0),
                   self.car.stop):
            try:
                fn()
            except Exception:
                pass
        self.steer, self.pan = 0, 0.0
        self.record("shutdown")

    # ------------------------------------------------------------ head
    def pan_to(self, target, dps):
        """Move the camera head to `target` degrees at `dps` degrees/second."""
        target = clamp(target, -PAN_LIMIT_DEG, PAN_LIMIT_DEG)
        step = max(1.0, dps * LOOP_S)
        while abs(target - self.pan) > 1e-6:
            self.estop.check()
            self.pan += clamp(target - self.pan, -step, step)
            self.car.set_cam_pan_angle(int(round(self.pan)))
            self.clock.sleep(LOOP_S)
        self.record("pan", angle=target)

    def scan(self, angle, dps, sweeps, hold):
        """Look left then right `sweeps` times, then face forward."""
        self.stop()
        for _ in range(sweeps):
            self.pan_to(-angle, dps)    # left
            self.wait(hold)
            self.pan_to(angle, dps)     # right
            self.wait(hold)
        self.pan_to(0, dps)
        self.record("scan", sweeps=sweeps, dps=dps)

    # ------------------------------------------------------------ wheels
    def _drive(self, direction, speed, steer):
        self.set_steer(steer)
        s = self.scaled(speed)
        if direction > 0:
            self.car.forward(s)
        else:
            self.car.backward(s)
        self.record("drive", direction=direction, speed=s, steer=self.steer)

    def approach(self, speed, detect_cm, timeout_s):
        """Drive straight until an obstacle is confirmed within `detect_cm`.

        Returns the distance at which it stopped. Invalid readings neither
        count towards nor reset the confirmation counter. Raises ProfileAbort
        if no obstacle shows up within `timeout_s`.
        """
        self._drive(+1, speed, 0)
        close = hard = 0
        end = self.clock.now() + timeout_s
        try:
            while True:
                self.estop.check()
                if self.clock.now() > end:
                    raise ProfileAbort("no obstacle detected within %.1fs" % timeout_s)
                d = self.read_distance()
                if d is not None:
                    close = close + 1 if d < detect_cm else 0
                    hard = hard + 1 if d < HARD_MIN_CM else 0
                    if close >= CONFIRM_READINGS or hard >= HARD_CONFIRM_READINGS:
                        self.stop()
                        self.record("detect", dist=d)
                        return d
                self.clock.sleep(LOOP_S)
        finally:
            self.car.stop()

    def move(self, direction, speed, seconds, steer=0, guard_cm=None):
        """Drive for `seconds` then stop. Forward moves with `guard_cm` stop
        early if an obstacle is confirmed that close. Returns False if
        stopped early. Reverse moves are unguarded (no rear sensor), so keep
        them short and the area behind the car clear."""
        self._drive(direction, speed, steer)
        end = self.clock.now() + seconds
        close = 0
        try:
            while True:
                self.estop.check()
                left = end - self.clock.now()
                if left <= 1e-6:
                    return True
                if direction > 0 and guard_cm:
                    d = self.read_distance()
                    if d is not None:
                        close = close + 1 if d < guard_cm else 0
                        if close >= CONFIRM_READINGS:
                            self.record("blocked", dist=d)
                            self.log("    forward move stopped early: obstacle at %.1f cm" % d)
                            return False
                    left = end - self.clock.now()
                    if left <= 1e-6:
                        return True
                self.clock.sleep(min(LOOP_S, left))
        finally:
            self.stop()
