"""Curve solver: recompute MAIN / ALT / CLOSE / the event re-plan for a new
layout or car model (brief 9.1).

Every curve has the same shape (all forward):

    lead-in straight → arc 1 (side1, R1, a1) → mid straight
                     → arc 2 (other side, R2, a2) → final straight

The arc-2 angle follows from the headings (start heading + arc 1 + arc 2 =
target heading). For a given (a1, R1, R2) the two free straights (mid and
final) enter the end position linearly, so they are solved exactly from a
2x2 system. A grid search over (a1, R1, R2) keeps the candidates that
satisfy every rule and picks, in order:

  1. the largest R2 (the long sweeping arc must read as ONE arc on camera),
  2. the shortest mid straight (no visible "wave" between the arcs),
  3. the largest obstacle clearance.
"""
import math
from dataclasses import dataclass

from . import geometry as g


@dataclass(frozen=True)
class Candidate:
    segments: tuple
    a1: float
    r1: float
    r2: float
    a2: float
    mid: float
    final: float
    clearance: float
    edge: float
    difference: float = 0.0     # max sideways distance from the reference path (if any)


def frange(lo, hi, step):
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 6) for i in range(n + 1)]


def two_arc(start, target, lead_in, side1, a1, r1, r2, offset):
    """Segments for one grid point, or None if the straights come out negative
    or the headings don't work out."""
    side2 = g.RIGHT if side1 == g.LEFT else g.LEFT
    s1 = g.heading_sign(g.FORWARD, side1)
    a2 = (start.h + s1 * a1 - target.h) * s1         # arc 2 undoes the rest of the heading
    if a2 <= 0:
        return None
    lead = [g.Segment(g.FORWARD, g.STRAIGHT, 0, lead_in)] if lead_in > 0 else []
    arc1 = g.Segment(g.FORWARD, side1, r1, a1)
    arc2 = g.Segment(g.FORWARD, side2, r2, a2)
    base = g.to_rear(g.end_pose(start, lead + [arc1, arc2], offset), offset)
    goal = g.to_rear(target, offset)
    h1 = math.radians(start.h + s1 * a1)              # heading on the mid straight
    h2 = math.radians(target.h)                       # heading on the final straight
    ux, uy, vx, vy = math.cos(h1), math.sin(h1), math.cos(h2), math.sin(h2)
    det = ux * vy - uy * vx
    if abs(det) < 1e-9:
        return None
    dx, dy = goal.x - base.x, goal.y - base.y
    mid = (dx * vy - dy * vx) / det
    final = (ux * dy - uy * dx) / det
    if mid < -1e-9 or final < -1e-9:
        return None
    return lead + [arc1, g.Segment(g.FORWARD, g.STRAIGHT, 0, max(mid, 0.0)),
                   arc2, g.Segment(g.FORWARD, g.STRAIGHT, 0, max(final, 0.0))]


def solve(start, target, obstacle, car, layout, *, side1, lead_in, min_clear,
          angles=None, r1s=None, r2s=None, max_mid=10.0, min_final=None,
          edge_margin=None, step=1.0, max_clear=None, reference=None):
    """Best Candidate, or None if nothing satisfies the rules.

    obstacle: polygon (e.g. geometry.box(*layout.o0, layout.obstacle_half)).
    min_final defaults to car.ramp_cm (room to slow down on a straight).
    max_clear (optional) keeps CLOSE in its band.
    reference (optional): a list of centre poses (e.g. MAIN's). When given, the
    objective becomes "largest sideways difference from the reference path"
    (geometry.path_difference), so ALT reads as a different curve on camera;
    ties go to the larger clearance."""
    angles = angles or frange(20, 60, 1)
    r1s = r1s or frange(max(car.min_radius, 30), 60, 5)
    r2s = r2s or frange(50, 250, 5)
    min_final = car.ramp_cm if min_final is None else min_final
    edge_margin = layout.edge_margin if edge_margin is None else edge_margin
    off = car.rear_to_centre

    cheap = []
    for a1 in angles:
        for r1 in r1s:
            for r2 in r2s:
                if r1 < car.min_radius or r2 < car.min_radius:
                    continue
                s = two_arc(start, target, lead_in, side1, a1, r1, r2, off)
                if s is None:
                    continue
                mid, final = s[-3].amount, s[-1].amount
                if mid > max_mid or final < min_final:
                    continue
                cheap.append((-r2, mid, a1, r1, s))
    cheap.sort(key=lambda c: c[:2])

    if reference is not None:
        return _solve_difference(cheap, start, obstacle, car, layout, reference, min_clear, max_clear,
                                 edge_margin, step)

    best, best_r2 = None, None
    for neg_r2, mid, a1, r1, s in cheap:
        if best_r2 is not None and -neg_r2 < best_r2:
            break                      # every remaining candidate has a smaller R2
        poses = [p for p, _ in g.sample(start, s, off, step)]
        e = g.edge_clearance(poses, car, layout.width, layout.height)
        if e < edge_margin:
            continue
        c = g.clearance(poses, obstacle, car)
        if c < min_clear or (max_clear is not None and c > max_clear):
            continue
        cand = Candidate(tuple(s), a1, r1, -neg_r2, s[-2].amount, mid, s[-1].amount, c, e)
        if best is None or (cand.mid, -cand.clearance) < (best.mid, -best.clearance):
            best, best_r2 = cand, -neg_r2
    return best


def _solve_difference(cands, start, obstacle, car, layout, reference, min_clear, max_clear, edge_margin, step):
    off = car.rear_to_centre
    best = None
    for neg_r2, mid, a1, r1, s in cands:
        poses = [p for p, _ in g.sample(start, s, off, step)]
        d = g.path_difference(poses, reference)
        if best is not None and d < best.difference - 1e-9:
            continue                   # cheap test first: can't win
        e = g.edge_clearance(poses, car, layout.width, layout.height)
        if e < edge_margin:
            continue
        c = g.clearance(poses, obstacle, car)
        if c < min_clear or (max_clear is not None and c > max_clear):
            continue
        cand = Candidate(tuple(s), a1, r1, -neg_r2, s[-2].amount, mid, s[-1].amount, c, e, d)
        if best is None or (round(d, 2), c) > (round(best.difference, 2), best.clearance):
            best = cand
    return best
