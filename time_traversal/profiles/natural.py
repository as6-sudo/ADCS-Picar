"""C - Natural: gentle pacing with slow drifts and occasional hesitations.

Speed drifts around BASE with two slow, incommensurate sine waves, so it never
quite repeats. Every few seconds the car eases down to a near halt, lingers
briefly and smoothly recovers. All changes are rate-limited by ACCEL, so
starts and stops are always gradual.
"""
import math
from ..core import Profile

BASE = 20
SWAY = 5              # +/- speed drift
ACCEL = 15.0          # speed units/s; caps every change
HESITATE_EVERY = (2.5, 4.0)   # seconds between hesitations
HESITATE_SPEED = 4
HESITATE_HOLD = (0.4, 0.8)    # seconds lingering at HESITATE_SPEED
EASE_OUT = 1.5        # seconds to wind down at the end of the run


class Natural(Profile):
    key = "C"
    name = "Natural"

    def reset(self):
        self.speed = 0.0
        self.next_hesitation = self.rng.uniform(*HESITATE_EVERY)
        self.hold_until = None

    def update(self, t, dist, dt):
        target = (BASE + SWAY * 0.6 * math.sin(2 * math.pi * t / 5.3)
                  + SWAY * 0.4 * math.sin(2 * math.pi * t / 2.9 + 1.0))

        if self.hold_until is None and t >= self.next_hesitation:
            target = HESITATE_SPEED
            if self.speed <= HESITATE_SPEED + 1:
                self.hold_until = t + self.rng.uniform(*HESITATE_HOLD)
        elif self.hold_until is not None:
            target = HESITATE_SPEED
            if t >= self.hold_until:
                self.hold_until = None
                self.next_hesitation = t + self.rng.uniform(*HESITATE_EVERY)

        if self.duration - t < EASE_OUT:
            target = min(target, BASE * (self.duration - t) / EASE_OUT)

        step = ACCEL * dt
        self.speed += max(-step, min(step, target - self.speed))
        return self.speed
