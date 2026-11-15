"""The physical layout: arena, targets, obstacle, tape strips, cue offsets.

Every number here is a design default from the brief (section 4 unless
noted). Change them here when the floor is taped differently; then run
`plan.py` (and `plan.py --solve` if the curves no longer fit).
"""
from dataclasses import dataclass, field

from .geometry import Pose


@dataclass(frozen=True)
class TapeStop:
    """A short dark strip across the path at target_x + grayscale_offset (brief 6.5)."""
    enabled: bool = True
    # The stop check is active only on the leg's final straight (default).
    # "last_n" mode instead arms it for the last `last_n_cm` of the leg,
    # useful if a leg must end on an arc. Approved fallback, default 12 cm.
    mode: str = "final_straight"            # "final_straight" | "last_n"
    last_n_cm: float = 12.0
    # In final_straight mode the check arms for at most the last
    # `arm_window_cm` of the straight, so a long final straight (C->A, 105 cm)
    # doesn't stop on the strip it starts on.
    arm_window_cm: float = 15.0
    # If no strip is seen this many cm past the expected stop, stop anyway
    # and log TAPE_MISSED (brief 6.5, value is a starting guess).
    miss_margin_cm: float = 4.0


@dataclass(frozen=True)
class Layout:
    width: float = 150.0                    # arena, cm (brief 4)
    height: float = 150.0
    targets: dict = field(default_factory=lambda: {
        "A": Pose(20.0, 75.0, 0.0),         # start
        "B": Pose(125.0, 35.0, 0.0),
        "C": Pose(125.0, 75.0, 0.0),
        "D": Pose(125.0, 115.0, 0.0),
    })
    # Obstacle: 15 x 15 cm footprint, >= 15 cm tall, flat hard sides.
    obstacle_half: float = 7.5
    o0: tuple = (80.0, 75.0)                # start centre
    o1: tuple = (80.0, 45.0)                # end centre: slides 30 cm in -y, stopped by a bead
    obstacle_slide_cm_s: float = 37.5       # simulator: 30 cm in ~0.8 s (user estimate, 2026-11-15); time a real pull

    # Clearance rules (brief 4 and 7).
    edge_margin: float = 5.0                # footprint stays this far inside the walls...
    lane_edge_margin: float = 4.5           # ...except the LH lanes, which sit at exactly 5.0
    min_clearance: dict = field(default_factory=lambda: {
        "MAIN": 10.0,       # brief 7 said 15; lowered to 10 for the re-solved MAIN with a lead-in (user decision)
        "ALT": 9.0,         # ALT is chosen for its difference from MAIN; it only needs >= 9 cm
        "CLOSE": 6.0, "EVENT": 10.0, "LANE": 12.0, "STRAIGHT": 10.0,
    })
    # Solver only: ceiling that keeps CLOSE visibly close (the brief's CLOSE is 6.8).
    max_clearance: dict = field(default_factory=lambda: {"CLOSE": 8.0})
    end_tolerance_cm: float = 0.5           # ideal-model end-pose error (brief 9.1)
    end_tolerance_deg: float = 0.5

    # LH lanes (brief 5).
    lh_pullout_cm: float = 10.0             # F 0 10 before the first pivot
    lh_lower_lane_y: float = 45.0           # A→C lane, passes under O0
    # Planned (never driven) last straight into C. 0 gives the best O0 clearance
    # of the planned detour (5.3 cm); it cannot reach the 12 cm lane rule with
    # 3-step pivots at R30, so plan.py reports it as a warning (plot only).
    lh_c_final_straight_cm: float = 0.0
    lh_detect_cm: float = 20.0              # sensor stop on the lower lane...
    lh_confirm_readings: int = 2            # ...on this many consecutive valid readings (was 3; user, 2026-11-15)
    # Stop anyway this far into the lane: NO_OBSTACLE. Was 15; raised to 20
    # (user, 2026-11-15) because the car rests ~14 cm in after a normal stop.
    lh_detect_cap_cm: float = 20.0
    lh_cap_min_clear: float = 8.0           # at the cap the car must stay this far from the slide strip

    # HH/HL obstacle event (brief 5).
    event_stop_travel_cm: float = 23.5      # fixed travel before the stop (not sensor-based)
    # HL only (user, 2026-11-15): a short lead-in straight so the smooth profile
    # doesn't start on a full-lock arc from standstill. The stop stays on a
    # straight (F 0 5 -> F R R30 30 -> F 0 4), so stopping +/-2 cm early/late
    # only moves the arrival at C ~1 cm sideways. HH keeps the brief's start.
    event_hl_lead_in_cm: float = 5.0
    event_hl_stop_travel_cm: float = 24.7   # 5 + 15.7 (arc) + 4.0
    event_late_cm: float = 27.0             # validated reading above this -> OBSTACLE_LATE

    # Tape stops per target (brief 6.5). Strips are dark tape across the path:
    # strip_width along x, strip_length along y (centred on the target's y).
    strip_width_cm: float = 2.5
    strip_length_cm: float = 12.0
    tape: dict = field(default_factory=lambda: {t: TapeStop() for t in "ABCD"})

    # Operator cue offsets in seconds, relative to the PULL cue (brief 8).
    operator_reaction_s: float = 0.2        # simulator only: cue -> line starts moving (user estimate)
    cue: dict = field(default_factory=lambda: {
        "HH": 0.3,          # robot starts the A→C drive 0.3 s after the cue
        # HL robot starts 0.3 s after the cue (user, 2026-11-15): with the 5 cm
        # lead-in the stop came 1.1 s after the cue, only ~0.1 s after the
        # obstacle arrives; +0.3 s puts the stop at ~1.4 s (brief 8).
        "HL": 0.3,
        "LH": 0.0,          # cue when the first pivot of A→C finishes
    })

    def target(self, name):
        return self.targets[name]


DEFAULT = Layout()
