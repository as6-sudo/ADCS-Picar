"""Driving styles: how keyboard input is turned into motion for each
behaviour profile of the storyboards.

The operator always chooses WHERE the car goes; the style decides HOW it
moves there (speed, ramps, steering smoothness, head, quirks). Speeds are
ground speeds in cm/s and go through the calibration tables (motion.py),
so the car's real range applies: the slowest it can drive is the speed of
command 1 (~22 cm/s on this car), the fastest ~41 cm/s.
"""
import math
from dataclasses import dataclass

INSTANT = math.inf


@dataclass(frozen=True)
class Style:
    key: str
    name: str
    cruise_cm_s: float              # W / S speed (reverse uses reverse_frac of it)
    accel_cm_s2: float              # speed ramp; INSTANT = jump to the target
    steer_max: float                # deg at full A / D
    steer_dps: float                # how fast the steering follows A / D (deg/s)
    head_dps: float                 # arrows, glances, looks
    reverse_frac: float = 1.0
    glance_deg: float = 35.0        # Q / E
    glance_hold_s: float = 0.5
    pivots: bool = False            # LH: A / D tap = 90 deg 3-step pivot, W / S drive straight only
    pause_after_stop_s: float = 0.0 # LH: hold still after every stop before driving again
    late_stop_s: float = 0.0        # LL: keeps rolling this long after W / S is released
    wobble_deg: float = 0.0         # LL: steering wobble amplitude...
    wobble_hz: float = 0.0          # ...and frequency
    speed_jitter: float = 0.0       # LL: +/- fraction of the speed, slowly varying
    head_drift_deg: float = 0.0     # LL: the head wanders by up to this much
    surprise: str = "backup"        # B: "backup" (HH/HL), "halt_retrace" (LH), "late_retrace" (LL)
    surprise_pause_s: float = 0.3
    look_after_surprise: bool = False
    wiggle_deg: float = 15.0
    wiggle_count: int = 3
    wiggle_period_s: float = 0.25


HH = Style("HH", "High/High: fast, precise, smooth", cruise_cm_s=41.0, accel_cm_s2=150.0,
           steer_max=30.0, steer_dps=150.0, head_dps=180.0, glance_hold_s=0.4,
           surprise="backup", surprise_pause_s=0.2, look_after_surprise=True)

HL = Style("HL", "High/Low: slow, smooth, gentle", cruise_cm_s=25.0, accel_cm_s2=12.0,
           steer_max=22.0, steer_dps=35.0, head_dps=60.0, glance_hold_s=0.8,
           surprise="backup", surprise_pause_s=0.6, look_after_surprise=True,
           wiggle_period_s=0.4)

LH = Style("LH", "Low/High: grid lanes, constant speed, dead stops", cruise_cm_s=35.0,
           accel_cm_s2=INSTANT, steer_max=0.0, steer_dps=INSTANT, head_dps=120.0,
           pivots=True, pause_after_stop_s=2.0, surprise="halt_retrace",
           surprise_pause_s=2.0, wiggle_deg=10.0)

LL = Style("LL", "Low/Low: wandering, wobbly, late", cruise_cm_s=28.0, accel_cm_s2=40.0,
           steer_max=25.0, steer_dps=60.0, head_dps=40.0, glance_hold_s=0.3,
           late_stop_s=1.0, wobble_deg=9.0, wobble_hz=0.6, speed_jitter=0.15,
           head_drift_deg=20.0, surprise="late_retrace", surprise_pause_s=0.5,
           wiggle_deg=12.0, wiggle_count=2, wiggle_period_s=0.5)

STYLES = {s.key: s for s in (HH, HL, LH, LL)}
ORDER = ("HH", "HL", "LH", "LL")        # keys 1-4
