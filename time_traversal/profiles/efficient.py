"""B - Efficient: a jump start, then keeps pushing to top speed.

The car launches hard: in the first fraction of a second it leaps from rest
to LAUNCH. From there it keeps climbing towards TOP on a curve that is
steepest early and flattens out, reaching TOP after TOP_AT of the run and
holding it. There are no pauses. Obstacles inside LOOKAHEAD shave the speed
off proportionally instead of stopping dead.
"""
from ..core import Profile, STOP_CM

LAUNCH = 65           # speed reached almost instantly off the line
TOP = 100
TOP_AT = 0.6          # fraction of the run at which TOP is reached
POWER = 0.5           # <1 front-loads the acceleration
LOOKAHEAD = 60        # cm; start trimming speed for an obstacle
RATE = 300.0          # max speed change per second (0 -> LAUNCH in ~0.2 s)


class Efficient(Profile):
    key = "B"
    name = "Efficient"

    def reset(self):
        self.speed = 0.0

    def update(self, t, dist, dt):
        frac = min(1.0, t / (TOP_AT * self.duration))
        target = LAUNCH + (TOP - LAUNCH) * frac ** POWER
        if dist < LOOKAHEAD:
            target *= 0.4 + 0.6 * max(0.0, dist - STOP_CM) / (LOOKAHEAD - STOP_CM)
        step = RATE * dt
        self.speed += max(-step, min(step, target - self.speed))
        return self.speed
