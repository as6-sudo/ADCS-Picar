"""Behaviour per profile: speeds, ramps, pauses, gaze, wiggle.

This is kept apart from the geometry (paths.py) so the behaviour can be
tuned without touching the paths (brief section 7). Speeds are ground
speeds in cm/s; executor/motion convert them to commands with the
calibration tables. Values are the brief's starting points or first guesses.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Behaviour:
    key: str
    name: str
    cruise_cm_s: float              # target ground speed (brief 7)
    # Speed ramp: fractions of cruise for the steps of the first/last straight
    # of each run (empty = instant starts and stops). Ramps never go inside an arc.
    ramp_fracs: tuple
    rolling: bool                   # True: steer while moving; False: stop -> steer -> settle -> go
    pause_at_target_s: float        # pause after arriving at a target
    gaze_before_legs: tuple = ()    # legs (numbers) before which the head looks at the target
    gaze_dps: float = 120.0         # head pan speed for gazes
    gaze_hold_s: float = 0.6
    look_at_obstacle: bool = False  # after the surprise back-up (HL)
    surprise_pause_s: float = 0.3   # stillness after the obstacle stop
    wiggle_at_end: bool = False     # "happy wiggle" after the last leg
    wiggle_deg: float = 15.0        # steering/pan amplitude of the wiggle
    wiggle_count: int = 3
    wiggle_period_s: float = 0.25


HH = Behaviour("HH", "Fast, precise, smooth", cruise_cm_s=50.0,
               ramp_fracs=(0.4, 0.6, 0.8, 1.0),          # short ramps: fast build-up
               rolling=True, pause_at_target_s=0.5,
               gaze_before_legs=(1, 5), gaze_dps=180.0, gaze_hold_s=0.4,
               surprise_pause_s=0.2, wiggle_at_end=True)

HL = Behaviour("HL", "Slow, smooth, gentle", cruise_cm_s=25.0,
               ramp_fracs=(0.7, 0.8, 0.9, 1.0),          # long, gentle ramps (min speed ~16 cm/s)
               rolling=True, pause_at_target_s=1.0,
               gaze_dps=60.0, gaze_hold_s=0.8, look_at_obstacle=True,
               surprise_pause_s=0.6, wiggle_at_end=True, wiggle_period_s=0.4)

LH = Behaviour("LH", "Constant speed, instant starts and stops", cruise_cm_s=35.0,
               ramp_fracs=(), rolling=False,
               pause_at_target_s=2.0)                    # brief 7: 2.0 s at each target

PROFILES = {b.key: b for b in (HH, HL, LH)}
