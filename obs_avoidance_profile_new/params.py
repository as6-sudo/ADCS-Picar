"""Every tunable number for the four obstacle-avoidance profiles lives here.

Conventions (checked against SunFounder picarx v2.0, the same library
`time_traversal` uses):
  * set_dir_servo_angle(+x) steers RIGHT, (-x) steers LEFT, limit +/-30 deg.
  * set_cam_pan_angle(+x) pans the camera head RIGHT, (-x) LEFT, limit +/-90.
  * forward(s)/backward(s) take 0..100. Inside picarx a non-zero speed s is
    sent to the motors as PWM = 50 + s/2 percent, so speed 25 is ~62 % PWM and
    speed 100 is 100 % PWM. Speed differences therefore look smaller on the
    car than the numbers suggest; the profiles use wide gaps on purpose.
  * get_distance() returns cm, or -1 (timeout) / -2 (bad echo).

Durations of turns are open-loop (there is no heading sensor), so the turn
angles depend on floor grip and battery level. Tune *_s values with
`calibrate.py` (steering check) before a full-speed run.
"""
from dataclasses import dataclass

# ---------------------------------------------------------------- shared
LOOP_S = 0.02               # control-loop / servo-step period (s)

# Obstacle detection, shared by A, B and C (D has a justified exception).
DETECT_CM = 30.0            # stop when an obstacle is confirmed closer than this
HARD_MIN_CM = 12.0          # never drive forward when closer than this, any profile
CONFIRM_READINGS = 3        # consecutive close readings needed to count as an obstacle
HARD_CONFIRM_READINGS = 2   # consecutive readings needed for the hard-minimum stop
VALID_MIN_CM = 2.0          # HC-SR04 blind zone; anything below is noise
VALID_MAX_CM = 400.0        # beyond rated range; treat as invalid
APPROACH_TIMEOUT_S = 8.0    # give up (and stop) if no obstacle is found in this time
EXIT_GUARD_CM = 25.0        # forward legs of the exit stop early if something is this close

PAN_LIMIT_DEG = 60          # keep the head well inside the +/-90 servo range
SERVO_SETTLE_S = 0.3        # wait after centring servos (same as time_traversal.home)

# Speed gating for the physical car (see run.py / calibrate.py)
DEFAULT_MAX_SAFE_SPEED = 70     # profile B speed until calibration sets max_safe_speed
UNCALIBRATED_SPEED_SCALE = 0.5  # all speeds are scaled by this until calibration passes


@dataclass(frozen=True)
class ProfileParams:
    key: str
    name: str
    # approach
    forward_speed: int              # 0..100; None-like 0 means "use calibrated max"
    detect_cm: float
    # head scanning (camera pan servo only)
    pan_angle: int                  # degrees to each side
    pan_speed_dps: float            # degrees per second while panning
    pan_hold_s: float               # dwell at each extreme
    scan_sweeps: int                # left-right sweeps per scan
    scan_cycles: int                # how many scan(+reverse) cycles
    # reversing (straight back, wheels centred)
    reverse_speed: int
    reverse_s: float
    pause_s: float                  # pause between movements
    # exit manoeuvre
    exit_maneuver: str              # "right_turn" | "right_uturn" | "none"
    turn_speed: int = 0
    turn_steer: int = 30            # magnitude of steering during turns
    turn_s: float = 0.0             # A: forward right arc
    uturn_reverse_s: float = 0.0    # B/C: reverse leg, wheels LEFT  -> nose swings right
    uturn_forward_s: float = 0.0    # B/C: forward leg, wheels RIGHT -> keeps swinging right
    exit_speed: int = 0
    exit_s: float = 0.0             # final straight drive out of the scene
    # D only: simulated forward attempts
    attempts: int = 0
    attempt_speed: int = 0
    recoil_s: float = 0.0           # small back-off before each attempt
    lunge_s: float = 0.0            # forward attempt (<= recoil_s so the car never creeps closer)


A = ProfileParams(
    key="A", name="Repetitive / highly reactive",
    forward_speed=40, detect_cm=DETECT_CM,
    pan_angle=35, pan_speed_dps=600, pan_hold_s=0.05,   # servo-limited: very rapid
    scan_sweeps=3, scan_cycles=3,
    reverse_speed=40, reverse_s=0.35, pause_s=0.15,
    exit_maneuver="right_turn",
    turn_speed=45, turn_steer=30, turn_s=1.05,
    exit_speed=45, exit_s=1.5,
)

B = ProfileParams(
    key="B", name="Fast / decisive",
    forward_speed=0,                 # 0 = highest calibrated safe speed
    detect_cm=DETECT_CM,
    pan_angle=35, pan_speed_dps=220, pan_hold_s=0.08,   # slower than A, still quick
    scan_sweeps=1, scan_cycles=1,
    reverse_speed=0, reverse_s=0.0, pause_s=0.08,
    exit_maneuver="right_uturn",
    turn_speed=70, turn_steer=30, uturn_reverse_s=0.8, uturn_forward_s=0.6,
    exit_speed=70, exit_s=1.2,
)

C = ProfileParams(
    key="C", name="Slow / cautious",
    forward_speed=25, detect_cm=DETECT_CM,
    pan_angle=35, pan_speed_dps=50, pan_hold_s=0.6,     # slow, deliberate look
    scan_sweeps=1, scan_cycles=2,
    reverse_speed=25, reverse_s=0.8, pause_s=0.9,
    exit_maneuver="right_uturn",
    turn_speed=30, turn_steer=30, uturn_reverse_s=1.25, uturn_forward_s=0.9,
    exit_speed=30, exit_s=2.0,
)

# D stops closer (20 cm) than the shared 30 cm on purpose: it should look like
# it is pressing against the obstacle. HARD_MIN_CM (12 cm) still applies.
D = ProfileParams(
    key="D", name="Persistent / non-avoidant",
    forward_speed=45, detect_cm=20.0,
    pan_angle=0, pan_speed_dps=0, pan_hold_s=0.0,       # no head scanning at all
    scan_sweeps=0, scan_cycles=0,
    reverse_speed=0, reverse_s=0.0, pause_s=0.3,
    exit_maneuver="none",
    attempts=6, attempt_speed=40, recoil_s=0.25, lunge_s=0.22,
)

PROFILES = {p.key: p for p in (A, B, C, D)}
