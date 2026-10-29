"""D - Erratic: random bursts, random pauses, no rhythm.

The run is a chain of random segments. Each one picks a random kind (burst,
crawl, pause or lurch), a random length and a random acceleration (anything
from an instant jump to a sluggish ramp), so neither the speeds nor the timing
ever settle into a pattern.

On top of that the car zigzags hard: the steering snaps to the opposite side
every ZIG_EVERY seconds, at a random angle within ZIG_ANGLE, so the path
weaves tightly left and right. Pauses are kept rare so most of the run is
spent weaving.
"""
from ..core import Profile

# kind: (weight, speed range, duration range in s)
SEGMENTS = {
    "burst": (4, (45, 80), (0.3, 1.0)),
    "crawl": (2, (8, 20), (0.4, 1.2)),
    "pause": (1, (0, 0), (0.2, 0.8)),
    "lurch": (2, (55, 90), (0.1, 0.3)),
}
RATES = (15.0, 60.0, 400.0, None)   # speed units/s; None = instant jump
ZIG_EVERY = (0.25, 0.5)   # seconds between steering flips
ZIG_ANGLE = (25, 30)      # degrees either side of straight


class Erratic(Profile):
    key = "D"
    name = "Erratic"

    def reset(self):
        self.speed = 0.0
        self.left = 0.0
        self.target = 0.0
        self.rate = None
        self.last_kind = None
        self.side = self.rng.choice((-1, 1))
        self.zig_left = 0.0

    def _new_segment(self):
        r = self.rng
        kinds = [k for k in SEGMENTS if k != self.last_kind]
        kind = r.choices(kinds, weights=[SEGMENTS[k][0] for k in kinds])[0]
        _, speeds, durations = SEGMENTS[kind]
        self.last_kind = kind
        self.target = r.uniform(*speeds)
        self.left = r.uniform(*durations)
        self.rate = r.choice(RATES)

    def update(self, t, dist, dt):
        self.left -= dt
        if self.left <= 0:
            self._new_segment()
        self.zig_left -= dt
        if self.zig_left <= 0:
            self.side = -self.side
            self.steer = self.side * self.rng.uniform(*ZIG_ANGLE)
            self.zig_left = self.rng.uniform(*ZIG_EVERY)
        if self.rate is None:
            self.speed = self.target
        else:
            step = self.rate * dt
            self.speed += max(-step, min(step, self.target - self.speed))
        return self.speed
