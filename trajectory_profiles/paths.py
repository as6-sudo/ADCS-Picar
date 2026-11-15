"""Path geometry: the curves, the LH lanes, the obstacle event, and the legs
of every profile (brief sections 5 and 7).

Curves are written for A→B. A→D is `mirror`, B→A is `retrace`.
Segment text format: "F R R30 45" = forward, right steering, rear-axle
radius 30 cm, 45° of heading change; "F 0 2.7" = forward straight 2.7 cm.

Nothing here drives the car; `plan.py` checks every leg and the executor
runs them.
"""
import math
from dataclasses import dataclass, field

from . import geometry as g
from .geometry import seg, segs

# ------------------------------------------------------------------ curves (A→B)
# The brief's MAIN (section 5). Kept for reference and tests; NOT driven,
# because it starts with an arc and leaves no straight to ramp speed up on.
MAIN_BRIEF = segs("F R R30 45 | F 0 2.7 | F L R100 45 | F 0 11.2")

# MAIN re-solved with a lead-in straight >= the ramp distance (user decision,
# 2026-11-14): `python trajectory_profiles/plan.py --solve` with lead_in = 8 cm
# (CarModel.ramp_cm), min clearance to O0 = 10 cm, max mid straight 10 cm.
# Result: O0 clearance 12.8 cm, edge 7.0 cm. The long arc is R85 (was R100).
MAIN = segs("F 0 8 | F R R30 49 | F 0 0.59 | F L R85 49 | F 0 9.82")

ALT = segs("F 0 5 | F R R40 45 | F 0 6.9 | F L R80 45 | F 0 10.3")         # brief 5, 11.1 cm
CLOSE = segs("F 0 10 | F R R40 47 | F 0 6.9 | F L R70 47 | F 0 9.9")       # brief 5, 6.8 cm (on purpose)
CURVES = {"MAIN": MAIN, "ALT": ALT, "CLOSE": CLOSE}

# C→A after the obstacle has moved (brief 5): 13 cm to O1.
C_TO_A = segs("B 0 105")


# ------------------------------------------------------------------ LH pivots
def right_pivot(r):
    """90° clockwise corner as a 3-step pivot at full lock (brief 5)."""
    return [g.Segment("F", "R", r, 30), g.Segment("B", "L", r, 30), g.Segment("F", "R", r, 30)]


def left_pivot(r):
    """90° counter-clockwise corner, starting in reverse (brief 5)."""
    return [g.Segment("B", "R", r, 30), g.Segment("F", "L", r, 30), g.Segment("B", "R", r, 30)]


def _straight(length):
    return g.Segment("F", "0", 0, round(length, 3))


def _delta(heading, segments, off):
    """Centre displacement of `segments` driven from heading `heading`."""
    e = g.end_pose(g.Pose(0.0, 0.0, heading), segments, off)
    return e.x, e.y


def lh_lane_ab(layout, car):
    """LH A→B: pull-out → right pivot → south → left pivot → east to B.

    The lane runs at the target's y (35 for B). Straight lengths are solved
    from the layout (the brief's 39.9 and 95.1 come out of this)."""
    off, r = car.rear_to_centre, car.min_radius
    a, b = layout.target("A"), layout.target("B")
    head = [_straight(layout.lh_pullout_cm)] + right_pivot(r)
    p = g.end_pose(a, head, off)
    dx, dy = _delta(-90.0, left_pivot(r), off)
    south = p.y - (b.y - dy)
    east = b.x - (p.x + dx)
    return head + [_straight(south)] + left_pivot(r) + [_straight(east)]


def lh_lane_ac_planned(layout, car):
    """LH A→C as planned: lower lane (y=45) under O0, north, east into C.

    Only the part up to the obstacle stop is ever driven; the rest is for
    plotting and documentation, built with the same primitives."""
    off, r = car.rear_to_centre, car.min_radius
    a, c = layout.target("A"), layout.target("C")
    head = [_straight(layout.lh_pullout_cm)] + right_pivot(r)
    p = g.end_pose(a, head, off)
    d1 = _delta(-90.0, left_pivot(r), off)
    south = p.y - (layout.lh_lower_lane_y - d1[1])
    lane_start = g.Pose(p.x + d1[0], layout.lh_lower_lane_y, 0.0)
    up = _delta(0.0, left_pivot(r), off)          # east → north
    over = _delta(90.0, right_pivot(r), off)      # north → east
    final = layout.lh_c_final_straight_cm
    east = c.x - final - over[0] - up[0] - lane_start.x
    north = c.y - over[1] - up[1] - lane_start.y
    lane = head + [_straight(south)] + left_pivot(r)
    return lane, lane + [_straight(east)] + left_pivot(r) + [_straight(north)] + right_pivot(r) + [_straight(final)]


def lh_lane_stop_cm(layout, car):
    """Expected distance into the lower lane at which the ultrasonic first
    reads below lh_detect_cm (ideal model, no reading latency), capped."""
    lane, _ = lh_lane_ac_planned(layout, car)
    start = g.end_pose(layout.target("A"), lane, car.rear_to_centre)
    face = layout.o1[0] - layout.obstacle_half
    d = face - layout.lh_detect_cm - car.ultrasonic_offset - start.x
    return min(max(d, 0.0), layout.lh_detect_cap_cm)


# ------------------------------------------------------------------ obstacle event (HH, HL)
EVENT_ARC = seg("F R R30 30")                                           # planned "under" path, brief 5.1
EVENT_BACKUP = segs("B R R30 15")                                       # surprise back-up, brief 5.4
EVENT_REPLAN = segs("F L R40 23 | F 0 33.5 | F R R120 8 | F 0 25.0")    # re-plan to C, brief 5.5


def event_approach(travel_cm):
    """The planned 'under' path, cut after `travel_cm` of travel (brief 5.2)."""
    return [EVENT_ARC, _straight(max(0.0, travel_cm - EVENT_ARC.length))]


def event_planned():
    """The full 'under' path, for plotting only."""
    return [EVENT_ARC, _straight(45.0)]


# ------------------------------------------------------------------ legs
@dataclass
class Phase:
    label: str
    segments: list
    obstacle: str            # "O0", "O1", "SWEEP" (moving: the strip it slides through) or "NONE"
    kind: str                # clearance rule key in Layout.min_clearance
    edge_margin: float = None


@dataclass
class Leg:
    number: int
    name: str
    start: str               # target name
    end: str                 # target name
    phases: list
    planned: list = field(default_factory=list)   # extra (start pose, segments) drawn but never driven
    planned_kind: str = None
    note: str = ""

    @property
    def segments(self):
        return [s for p in self.phases for s in p.segments]


def legs(profile, layout, car):
    """The legs of one profile, in take order (brief section 7)."""
    A = layout.target("A")
    lane_b = lh_lane_ab(layout, car)
    lane_d = g.mirror(lane_b)
    ev_driven = event_approach(layout.event_stop_travel_cm)

    def event(n):
        return Leg(n, "A->C event", "A", "C", [
            Phase("approach (obstacle moving)", ev_driven, "SWEEP", "EVENT"),
            Phase("surprise back-up", EVENT_BACKUP, "O1", "EVENT"),
            Phase("re-plan to C", EVENT_REPLAN, "O1", "EVENT")],
            planned=[(A, event_planned())], planned_kind=None,
            note="stop after %.1f cm of travel, then back up and re-plan" % layout.event_stop_travel_cm)

    if profile == "HH":
        return [
            Leg(1, "A->B MAIN", "A", "B", [Phase("MAIN", MAIN, "O0", "MAIN")]),
            Leg(2, "B->A MAIN retraced", "B", "A", [Phase("MAIN retraced", g.retrace(MAIN), "O0", "MAIN")]),
            event(3),
            Leg(4, "C->A straight reverse", "C", "A", [Phase("reverse", C_TO_A, "O1", "STRAIGHT")]),
            Leg(5, "A->D MAIN mirrored", "A", "D", [Phase("MAIN mirrored", g.mirror(MAIN), "O1", "MAIN")]),
        ]
    if profile == "HL":
        return [
            Leg(1, "A->D MAIN mirrored", "A", "D", [Phase("MAIN mirrored", g.mirror(MAIN), "O0", "MAIN")]),
            Leg(2, "D->A ALT mirrored, retraced", "D", "A",
                [Phase("ALT mirrored retraced", g.retrace(g.mirror(ALT)), "O0", "ALT")]),
            Leg(3, "A->B MAIN", "A", "B", [Phase("MAIN", MAIN, "O0", "MAIN")]),
            Leg(4, "B->A CLOSE retraced", "B", "A", [Phase("CLOSE retraced", g.retrace(CLOSE), "O0", "CLOSE")]),
            event(5),
        ]
    if profile == "LH":
        lane_m = layout.lane_edge_margin
        to_lane, planned_c = lh_lane_ac_planned(layout, car)
        driven = to_lane + [_straight(lh_lane_stop_cm(layout, car))]
        return [
            Leg(1, "A->B lane", "A", "B", [Phase("lane", lane_b, "O0", "LANE", lane_m)]),
            Leg(2, "B->A lane retraced", "B", "A", [Phase("lane retraced", g.retrace(lane_b), "O0", "LANE", lane_m)]),
            Leg(3, "A->C lane, obstacle stop, retrace to A", "A", "A", [
                Phase("lane to obstacle (obstacle moving)", driven, "SWEEP", "LANE", lane_m),
                Phase("retrace to A", g.retrace(driven), "O1", "LANE", lane_m)],
                planned=[(A, planned_c)], planned_kind="LANE",
                note="sensor stop below %.0f cm (3 readings), cap %.0f cm into the lane"
                     % (layout.lh_detect_cm, layout.lh_detect_cap_cm)),
            Leg(4, "A->D lane mirrored", "A", "D", [Phase("lane mirrored", lane_d, "O1", "LANE", lane_m)]),
        ]
    if profile == "LL":
        raise NotImplementedError("Profile LL is out of scope for this framework (brief section 1).")
    raise KeyError(profile)


PROFILE_KEYS = ("HH", "HL", "LH")      # LL is registered in legs() as a NotImplementedError stub


def obstacle_polygons(state, layout):
    h = layout.obstacle_half
    if state == "O0":
        return [g.box(*layout.o0, h)]
    if state == "O1":
        return [g.box(*layout.o1, h)]
    if state == "SWEEP":
        (x0, y0), (x1, y1) = layout.o0, layout.o1
        return [g.box((x0 + x1) / 2, (y0 + y1) / 2, h + abs(x1 - x0) / 2, h + abs(y1 - y0) / 2)]
    return []


def ultrasonic_reading(pose, layout, car, obstacle="O1", cone_deg=15.0):
    """Ideal ultrasonic distance (cm) from a pose to the obstacle: the
    shortest ray in a ±cone around the heading; None if nothing is hit."""
    h = layout.obstacle_half
    cx, cy = layout.o1 if obstacle == "O1" else layout.o0
    x0, x1, y0, y1 = cx - h, cx + h, cy - h, cy + h
    th = math.radians(pose.h)
    ox, oy = pose.x + car.ultrasonic_offset * math.cos(th), pose.y + car.ultrasonic_offset * math.sin(th)
    best = None
    for k in range(-3, 4):
        a = th + math.radians(cone_deg * k / 3)
        dx, dy = math.cos(a), math.sin(a)
        tmin, tmax = 0.0, float("inf")
        hit = True
        for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    hit = False
                    break
            else:
                t1, t2 = (lo - o) / d, (hi - o) / d
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
                if tmin > tmax:
                    hit = False
                    break
        if hit and (best is None or tmin < best):
            best = tmin
    return best
