"""Segments, poses and clearance geometry (ideal bicycle model).

Conventions (brief section 3):
  * units are cm and degrees; the arena origin is its bottom-left corner,
    +x points from A toward the targets, +y points up toward D;
  * a Pose is the car's GEOMETRIC CENTRE plus its heading; heading 0 faces
    +x and positive headings are counter-clockwise;
  * the kinematics run on the REAR-AXLE CENTRE (bicycle model). The car
    centre is `car.rear_to_centre` cm ahead of it. Turning radii are radii
    of the rear-axle centre's path (this is what reproduces the brief's
    numbers);
  * a Segment is (direction F/B, steering side L/R/0, radius, amount).
    Straights: amount = length in cm. Arcs: amount = heading change in
    degrees (always positive; the sign comes from `heading_sign`).

This module is pure math: no picarx, no timing.
"""
import math
from collections import namedtuple
from dataclasses import dataclass

FORWARD, BACKWARD = "F", "B"
LEFT, RIGHT, STRAIGHT = "L", "R", "0"


def heading_sign(direction, side):
    """THE sign rule, in exactly one place.

    Forward + left steering turns counter-clockwise (heading +).
    Reverse + left steering turns clockwise (heading -).
    Right steering is the opposite of left. Straight is 0."""
    d = {FORWARD: 1, BACKWARD: -1}[direction]
    s = {LEFT: 1, RIGHT: -1, STRAIGHT: 0}[side]
    return d * s


@dataclass(frozen=True)
class Segment:
    direction: str          # "F" or "B"
    side: str               # "L", "R" or "0"
    radius: float           # rear-axle turning radius in cm; 0 for straights
    amount: float           # cm for straights, degrees of heading change for arcs

    def __post_init__(self):
        if self.direction not in (FORWARD, BACKWARD):
            raise ValueError("direction must be F or B: %r" % (self.direction,))
        if self.side not in (LEFT, RIGHT, STRAIGHT):
            raise ValueError("side must be L, R or 0: %r" % (self.side,))
        if self.amount < 0:
            raise ValueError("amount must be >= 0 (the sign comes from the sign rule)")
        if self.side != STRAIGHT and self.radius <= 0:
            raise ValueError("an arc needs a positive radius")

    @property
    def is_arc(self):
        return self.side != STRAIGHT

    @property
    def length(self):
        """Path length of the rear-axle centre, cm."""
        return self.radius * math.radians(self.amount) if self.is_arc else self.amount

    @property
    def heading_change(self):
        """Signed heading change in degrees."""
        return heading_sign(self.direction, self.side) * self.amount if self.is_arc else 0.0

    def with_amount(self, amount):
        return Segment(self.direction, self.side, self.radius, amount)

    def cut(self, length):
        """The same segment, driven for only `length` cm of path."""
        length = max(0.0, min(length, self.length))
        if self.is_arc:
            return self.with_amount(math.degrees(length / self.radius))
        return self.with_amount(length)

    def __str__(self):
        if self.is_arc:
            return "%s %s R%g %g" % (self.direction, self.side, self.radius, round(self.amount, 2))
        return "%s 0 %g" % (self.direction, round(self.amount, 2))


def seg(text):
    """Parse one segment written like the brief: 'F R R30 45' or 'F 0 2.7'."""
    parts = text.replace("°", "").split()
    if len(parts) == 3 and parts[1] == STRAIGHT:
        return Segment(parts[0], STRAIGHT, 0.0, float(parts[2]))
    if len(parts) == 4 and parts[2].upper().startswith("R"):
        return Segment(parts[0], parts[1], float(parts[2][1:]), float(parts[3]))
    raise ValueError("cannot parse segment %r" % text)


def segs(text):
    """Parse a path written as 'F R R30 45 | F 0 2.7 | ...'."""
    return [seg(t) for t in text.split("|") if t.strip()]


def fmt(segments):
    return " -> ".join(str(s) for s in segments)


def mirror(segments):
    """A→B becomes A→D: swap L and R in every segment."""
    swap = {LEFT: RIGHT, RIGHT: LEFT, STRAIGHT: STRAIGHT}
    return [Segment(s.direction, swap[s.side], s.radius, s.amount) for s in segments]


def retrace(segments):
    """A→B becomes B→A: reverse the order and flip F↔B, keeping each
    segment's steering side and radius. In the ideal model this drives
    exactly back along the same path."""
    flip = {FORWARD: BACKWARD, BACKWARD: FORWARD}
    return [Segment(flip[s.direction], s.side, s.radius, s.amount) for s in reversed(segments)]


def total_length(segments):
    return sum(s.length for s in segments)


# ------------------------------------------------------------------ poses
Pose = namedtuple("Pose", "x y h")      # car centre (cm), heading (deg, CCW from +x)


def to_rear(pose, offset):
    h = math.radians(pose.h)
    return Pose(pose.x - offset * math.cos(h), pose.y - offset * math.sin(h), pose.h)


def to_centre(rear, offset):
    h = math.radians(rear.h)
    return Pose(rear.x + offset * math.cos(h), rear.y + offset * math.sin(h), rear.h)


def advance_rear(rear, segment, length=None):
    """Exact rear-axle pose after driving `length` cm (default: all) of `segment`."""
    s = segment.length if length is None else length
    signed = s if segment.direction == FORWARD else -s
    h = math.radians(rear.h)
    if not segment.is_arc:
        return Pose(rear.x + signed * math.cos(h), rear.y + signed * math.sin(h), rear.h)
    # curvature of the path: +1/R for left steering, -1/R for right; the
    # direction sign is carried by `signed`, so together they give the sign rule.
    k = heading_sign(FORWARD, segment.side) / segment.radius
    h2 = h + k * signed
    return Pose(rear.x + (math.sin(h2) - math.sin(h)) / k,
                rear.y + (math.cos(h) - math.cos(h2)) / k,
                math.degrees(h2))


def end_pose(start, segments, offset):
    """Car-centre pose after driving `segments` from car-centre pose `start`."""
    r = to_rear(start, offset)
    for s in segments:
        r = advance_rear(r, s)
    return to_centre(r, offset)


def sample(start, segments, offset, step=0.5):
    """Car-centre poses every `step` cm of rear-axle path (plus every segment end).

    Returns a list of (pose, segment_index)."""
    r = to_rear(start, offset)
    out = [(start, -1)]
    for i, s in enumerate(segments):
        n = max(1, int(math.ceil(s.length / step)))
        for j in range(1, n + 1):
            out.append((to_centre(advance_rear(r, s, s.length * j / n), offset), i))
        r = advance_rear(r, s)
    return out


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def pose_error(a, b):
    """(position error cm, heading error deg) between two poses."""
    return math.hypot(a.x - b.x, a.y - b.y), abs(wrap180(a.h - b.h))


# ------------------------------------------------------------------ footprint
def footprint(pose, car):
    """The car's 4 corners (CCW) for a car-centre pose."""
    h = math.radians(pose.h)
    c, s = math.cos(h), math.sin(h)
    back = car.rear - car.rear_to_centre        # along-car coordinates measured from the centre
    front = car.front - car.rear_to_centre
    w = car.half_width
    return [(pose.x + a * c - b * s, pose.y + a * s + b * c)
            for a, b in ((back, -w), (front, -w), (front, w), (back, w))]


def box(cx, cy, half_x, half_y=None):
    """Axis-aligned rectangle as 4 corners (CCW)."""
    hy = half_x if half_y is None else half_y
    return [(cx - half_x, cy - hy), (cx + half_x, cy - hy), (cx + half_x, cy + hy), (cx - half_x, cy + hy)]


def _point_segment(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L2 = ax * ax + ay * ay
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L2))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def _overlap(p, q):
    """Separating-axis test for two convex polygons."""
    for poly in (p, q):
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = a[1] - b[1], b[0] - a[0]
            pp = [nx * x + ny * y for x, y in p]
            qq = [nx * x + ny * y for x, y in q]
            if max(pp) < min(qq) or max(qq) < min(pp):
                return False
    return True


def polygon_distance(p, q):
    """Distance between two convex polygons; 0 if they overlap."""
    if _overlap(p, q):
        return 0.0
    best = float("inf")
    for P, Q in ((p, q), (q, p)):
        for pt in P:
            for i in range(len(Q)):
                best = min(best, _point_segment(pt, Q[i], Q[(i + 1) % len(Q)]))
    return best


def clearance(poses, polygon, car):
    """Smallest gap between the car footprint and `polygon` along `poses`."""
    return min(polygon_distance(footprint(p, car), polygon) for p in poses)


def edge_clearance(poses, car, width, height):
    """Smallest gap between the car footprint and the arena walls (negative = outside)."""
    best = float("inf")
    for p in poses:
        for x, y in footprint(p, car):
            best = min(best, x, y, width - x, height - y)
    return best
