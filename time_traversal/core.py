"""Shared runner, sensor filtering and a mock car for dry runs.

Every profile drives forward only. Profiles mostly differ in their speed over
time; a profile may also steer by setting `self.steer` (degrees, clamped to
+/-MAX_STEER), which D uses to zigzag and A uses for small fidgets.

The ultrasonic sensor is fixed to the front bumper, so the runner applies one
shared safety rule: if something is closer than STOP_CM the car holds still
until the way is clear. The sensor throws occasional false short echoes while
the motors and servos are running, so readings are median-filtered and the
car only counts as blocked after BLOCK_TICKS close readings in a row. It never reverses, since nothing behind the car can
be sensed.

All servos (steering, camera pan, camera tilt) are put in the home position
before and after every run.
"""
import random
import time

RUN_SECONDS = 10.0
TICK = 0.05          # control loop period (s)
MAX_SPEED = 100
NO_READING = 300.0   # treat timeouts (-1/-2) as "nothing ahead"
STOP_CM = 20.0       # hold still while an obstacle is closer than this
MAX_STEER = 30       # steering servo limit (degrees)
BLOCK_TICKS = 3      # consecutive close readings before holding still
CLEAR_CM = 25.0      # must read farther than this to count as clear again


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class Profile:
    """Base class. Subclasses return a forward speed (0..100) each tick.

    `self.duration` holds the run length so a profile can ease out before
    the end; `self.rng` is seeded from --seed for repeatable runs. Set
    `self.steer` to turn the front wheels (0 = straight).
    """
    key = "?"
    name = "base"
    duration = RUN_SECONDS
    steer = 0

    def __init__(self, seed=None):
        self.rng = random.Random(seed)

    def reset(self):
        pass

    def update(self, t, dist, dt):
        raise NotImplementedError


class DistanceFilter:
    """Median of the last 5 valid ultrasonic readings."""

    def __init__(self):
        self.buf = []

    def push(self, raw):
        if raw is None or raw <= 0:
            raw = self.buf[-1] if self.buf else NO_READING
        self.buf = (self.buf + [float(raw)])[-5:]
        return sorted(self.buf)[len(self.buf) // 2]


class MockCar:
    """Stand-in for Picarx so profiles can be exercised without hardware."""

    def __init__(self, verbose=True):
        self.verbose = verbose
        self.t0 = time.monotonic()
        self.angle = 0
        self.speed = 0

    def get_distance(self):
        # obstacle appears ahead periodically
        t = time.monotonic() - self.t0
        return 15.0 if 3.0 < t % 6.0 < 3.6 else 120.0

    def set_dir_servo_angle(self, a):
        self.angle = a

    def set_cam_pan_angle(self, a):
        pass

    def set_cam_tilt_angle(self, a):
        pass

    def forward(self, s):
        self.speed = s

    def backward(self, s):
        self.speed = -s

    def stop(self):
        self.speed = 0


def home(car):
    """Stop the motors and centre every servo (steering, camera pan and tilt)."""
    car.stop()
    car.set_dir_servo_angle(0)
    car.set_cam_pan_angle(0)
    car.set_cam_tilt_angle(0)
    time.sleep(0.3)            # let the servos reach home before moving on


def run_profile(profile, car, duration=RUN_SECONDS, log=True):
    """Drive `car` forward with `profile` for exactly `duration` seconds."""
    filt = DistanceFilter()
    profile.duration = duration
    profile.reset()
    home(car)
    start = last = time.monotonic()
    last_log = -1.0
    angle = 0
    blocked = False
    close = 0
    try:
        while True:
            now = time.monotonic()
            t = now - start
            if t >= duration:
                break
            dt = now - last
            last = now
            dist = filt.push(car.get_distance())
            speed = clamp(int(round(profile.update(t, dist, dt))), 0, MAX_SPEED)
            new_angle = clamp(int(round(profile.steer)), -MAX_STEER, MAX_STEER)
            if new_angle != angle:
                angle = new_angle
                car.set_dir_servo_angle(angle)
            close = close + 1 if dist < STOP_CM else 0
            if close >= BLOCK_TICKS:
                blocked = True
            elif dist > CLEAR_CM:
                blocked = False
            if speed > 0 and not blocked:
                car.forward(speed)
            else:
                car.stop()
            if log and t - last_log >= 0.25:
                last_log = t
                note = "  BLOCKED" if blocked else ""
                print(f"[{profile.key}] t={t:5.2f}s dist={dist:6.1f}cm "
                      f"speed={speed:4d} steer={angle:+3d}{note}")
            time.sleep(max(0.0, TICK - (time.monotonic() - now)))
    finally:
        home(car)
    print(f"[{profile.key}] {profile.name} finished after {duration:.0f}s")
