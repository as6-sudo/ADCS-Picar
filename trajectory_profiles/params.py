"""Behaviour per profile: speeds, ramps, pauses, gaze, wiggle.

This is kept apart from the geometry (paths.py) so the behaviour can be
tuned without touching the paths (brief section 7). Only the fields used
so far are here; the executor adds the rest.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Behaviour:
    key: str
    name: str
    cruise_cm_s: float          # target ground speed; converted to commands with the speed table
    ramps: bool                 # speed ramps on the first/last straight of each leg
    steer_while_rolling: bool   # False: stop -> steer -> settle -> go (brief 6.3)
    pause_at_target_s: float


# Target speeds are the brief's starting points (section 7).
HH = Behaviour("HH", "Fast, precise, smooth", cruise_cm_s=50.0, ramps=True,
               steer_while_rolling=True, pause_at_target_s=0.5)
HL = Behaviour("HL", "Slow, smooth, gentle", cruise_cm_s=25.0, ramps=True,
               steer_while_rolling=True, pause_at_target_s=1.0)
LH = Behaviour("LH", "Constant speed, instant starts and stops", cruise_cm_s=35.0, ramps=False,
               steer_while_rolling=False, pause_at_target_s=2.0)

PROFILES = {b.key: b for b in (HH, HL, LH)}
