"""A - Mechanical: jerky start-stop moves broken up by twitchy fidgets.

The car runs through a fixed, uneven sequence of steps: short moves chopped
up by frequent abrupt stops, and fidgets while stopped (quick on/off motor
blips and bursts of steering twitches). The twitches only happen while the
wheels are still, so they look busy without bending the path; the steering
is always back at 0 before the car moves again. Every change is instant,
with no ramps. The steps come at irregular moments, but the whole sequence
repeats exactly every cycle.
"""
from ..core import Profile

SPEED = 40
BLIP = 30       # speed of a fidget blip
TWITCH = 20     # degrees of a fidget steering twitch
T = 0.09        # seconds per twitch

# (seconds, speed, steering)
SEQUENCE = [
    (0.35, SPEED, 0),
    (0.15, 0, 0),
    (T, 0, TWITCH),           # fidget: left-right-left twitch
    (T, 0, -TWITCH),
    (T, 0, TWITCH),
    (0.15, 0, 0),             # re-centre before moving
    (0.25, SPEED, 0),
    (0.20, 0, 0),
    (0.08, BLIP, 0),          # fidget: blip, pause, blip
    (0.12, 0, 0),
    (0.08, BLIP, 0),
    (0.15, 0, 0),
    (0.40, SPEED, 0),
    (0.12, 0, 0),
    (T, 0, -TWITCH),          # fidget: long twitch burst
    (T, 0, TWITCH),
    (T, 0, -TWITCH),
    (T, 0, TWITCH),
    (T, 0, -TWITCH),
    (0.15, 0, 0),
    (0.20, SPEED, 0),
    (0.25, 0, 0),
    (0.30, SPEED, 0),
    (0.10, 0, 0),
    (T, 0, TWITCH),           # fidget: single twitch
    (0.15, 0, 0),
    (0.15, SPEED, 0),
    (0.15, 0, 0),
    (0.06, BLIP, 0),
    (0.10, 0, 0),
    (T, 0, -TWITCH),          # fidget: twitch pair
    (T, 0, TWITCH),
    (0.20, 0, 0),
    (0.45, SPEED, 0),
    (0.30, 0, 0),
]
CYCLE = sum(step[0] for step in SEQUENCE)


class Mechanical(Profile):
    key = "A"
    name = "Mechanical"

    def update(self, t, dist, dt):
        x = t % CYCLE
        for seconds, speed, steer in SEQUENCE:
            if x < seconds:
                self.steer = steer
                return speed
            x -= seconds
        self.steer = 0
        return 0
