"""Geometry, path and planner tests (laptop, standard library only):

    python -m unittest discover -s trajectory_profiles/tests -t .
"""
import math
import os
import sys
import unittest
from dataclasses import replace
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from trajectory_profiles import geometry as g, paths, solver  # noqa: E402
from trajectory_profiles.calibration import DEFAULT_CAR as CAR, car_model  # noqa: E402
from trajectory_profiles.layout import DEFAULT as LAYOUT, TapeStop  # noqa: E402
from trajectory_profiles.plan import check_profile  # noqa: E402

OFF = CAR.rear_to_centre
A, B, C, D = (LAYOUT.target(t) for t in "ABCD")
O0 = paths.obstacle_polygons("O0", LAYOUT)[0]
O1 = paths.obstacle_polygons("O1", LAYOUT)[0]


def poses(start, s):
    return [p for p, _ in g.sample(start, s, OFF, 0.5)]


def clear(start, s, obstacle):
    return g.clearance(poses(start, s), obstacle, CAR)


class SignRules(unittest.TestCase):
    def test_heading_sign_table(self):
        self.assertEqual(g.heading_sign("F", "L"), +1)
        self.assertEqual(g.heading_sign("F", "R"), -1)
        self.assertEqual(g.heading_sign("B", "L"), -1)
        self.assertEqual(g.heading_sign("B", "R"), +1)
        self.assertEqual(g.heading_sign("F", "0"), 0)

    def test_forward_left_turns_ccw_and_reverse_left_turns_cw(self):
        start = g.Pose(0, 0, 0)
        self.assertAlmostEqual(g.end_pose(start, [g.seg("F L R30 45")], OFF).h, 45)
        self.assertAlmostEqual(g.end_pose(start, [g.seg("B L R30 45")], OFF).h, -45)
        self.assertAlmostEqual(g.end_pose(start, [g.seg("F R R30 45")], OFF).h, -45)
        self.assertAlmostEqual(g.end_pose(start, [g.seg("B R R30 45")], OFF).h, 45)

    def test_quarter_circle_of_the_rear_axle(self):
        r = g.advance_rear(g.Pose(0, 0, 0), g.seg("F L R30 90"))
        self.assertAlmostEqual(r.x, 30)
        self.assertAlmostEqual(r.y, 30)
        # Backing up with right steering: the turning centre is on the car's
        # right at (0, -30); the car swings back around it and the nose ends up
        # pointing +90 (left).
        r = g.advance_rear(g.Pose(0, 0, 0), g.seg("B R R30 90"))
        self.assertAlmostEqual(r.x, -30)
        self.assertAlmostEqual(r.y, -30)
        self.assertAlmostEqual(r.h, 90)

    def test_straights_and_centre_offset(self):
        e = g.end_pose(g.Pose(10, 10, 90), g.segs("F 0 5 | B 0 2"), OFF)
        self.assertAlmostEqual(e.x, 10)
        self.assertAlmostEqual(e.y, 13)
        self.assertEqual(g.to_rear(g.Pose(20, 75, 0), 9), g.Pose(11, 75, 0))

    def test_parse_and_length(self):
        s = g.seg("F R R30 45")
        self.assertEqual((s.direction, s.side, s.radius, s.amount), ("F", "R", 30.0, 45.0))
        self.assertAlmostEqual(s.length, 30 * math.pi / 4)
        self.assertEqual(g.seg("B 0 105").length, 105)
        with self.assertRaises(ValueError):
            g.Segment("F", "L", 0, 10)


class MirrorRetrace(unittest.TestCase):
    def test_mirror_swaps_sides_and_lands_on_d(self):
        m = g.mirror(paths.MAIN)
        self.assertEqual([s.side for s in m], [{"L": "R", "R": "L"}.get(s.side, s.side) for s in paths.MAIN])
        dpos, dh = g.pose_error(g.end_pose(A, m, OFF), D)
        self.assertLess(dpos, 0.5)

    def test_retrace_flips_direction_and_reverses_order(self):
        r = g.retrace(paths.MAIN)
        self.assertEqual([s.direction for s in r], ["B"] * len(paths.MAIN))
        self.assertEqual(r[0], g.Segment("B", "0", 0, paths.MAIN[-1].amount))
        self.assertEqual(g.retrace(r), paths.MAIN)

    def test_leg_then_retrace_returns_to_start(self):
        cases = list(paths.CURVES.values()) + [paths.MAIN_BRIEF, paths.lh_lane_ab(LAYOUT, CAR),
                                                paths.EVENT_REPLAN, paths.right_pivot(30), paths.left_pivot(30)]
        for s in cases:
            for path in (s, g.mirror(s)):
                end = g.end_pose(A, path, OFF)
                back = g.end_pose(end, g.retrace(path), OFF)
                dpos, dh = g.pose_error(back, A)
                self.assertLess(dpos, 0.5, g.fmt(path))
                self.assertLess(dh, 0.5)

    def test_retrace_follows_the_same_path(self):
        out = poses(A, paths.MAIN)
        back = poses(g.end_pose(A, paths.MAIN, OFF), g.retrace(paths.MAIN))
        for p, q in zip(out, reversed(back)):
            self.assertLess(math.hypot(p.x - q.x, p.y - q.y), 0.5)


class BriefNumbersReproduce(unittest.TestCase):
    """The brief's design values (section 5) in our model."""

    def test_brief_main_alt_close_reach_b_with_their_clearances(self):
        for s, brief_clear, lo in ((paths.MAIN_BRIEF, 15.8, 15), (paths.ALT_BRIEF, 11.1, 10), (paths.CLOSE, 6.8, 6)):
            self.assertLess(g.pose_error(g.end_pose(A, s, OFF), B)[0], 0.5)
            c = clear(A, s, O0)
            self.assertGreaterEqual(c, lo)
            self.assertAlmostEqual(c, brief_clear, delta=0.3)

    def test_solver_recovers_main_straights(self):
        s = solver.two_arc(A, B, 0, "R", 45, 30, 100, OFF)
        self.assertAlmostEqual(s[-3].amount, 2.7, delta=0.05)
        self.assertAlmostEqual(s[-1].amount, 11.2, delta=0.06)

    def test_lh_lane_straights(self):
        lane = paths.lh_lane_ab(LAYOUT, CAR)
        self.assertAlmostEqual(lane[4].amount, 39.9, delta=0.15)
        self.assertAlmostEqual(lane[-1].amount, 95.1, delta=0.15)
        to_lane, _ = paths.lh_lane_ac_planned(LAYOUT, CAR)
        self.assertAlmostEqual(to_lane[4].amount, 29.9, delta=0.15)

    def test_c_to_a(self):
        self.assertLess(g.pose_error(g.end_pose(C, paths.C_TO_A, OFF), A)[0], 0.5)
        self.assertAlmostEqual(clear(C, paths.C_TO_A, O1), 13.0, delta=0.2)

    def test_mirrored_main_far_from_o1(self):
        self.assertGreater(clear(A, g.mirror(paths.MAIN_BRIEF), O1), 33)


class ResolvedMain(unittest.TestCase):
    def test_main_has_a_ramp_lead_in_and_meets_the_rules(self):
        self.assertFalse(paths.MAIN[0].is_arc)
        self.assertGreaterEqual(paths.MAIN[0].length, CAR.ramp_cm)
        self.assertFalse(paths.MAIN[-1].is_arc)
        self.assertGreaterEqual(paths.MAIN[-1].length, CAR.ramp_cm)
        self.assertGreaterEqual(clear(A, paths.MAIN, O0), LAYOUT.min_clearance["MAIN"])
        self.assertLess(g.pose_error(g.end_pose(A, paths.MAIN, OFF), B)[0], 0.5)

    def test_main_still_reads_as_one_long_arc(self):
        arcs = [s for s in paths.MAIN if s.is_arc]
        self.assertEqual(len(arcs), 2)
        self.assertGreater(arcs[1].radius, 2 * arcs[0].radius)   # tight pull-out, long sweep
        self.assertLess(paths.MAIN[2].length, 3)                 # almost no straight between them

    def test_solver_regression(self):
        c = solver.solve(A, B, O0, CAR, LAYOUT, side1="R", lead_in=CAR.ramp_cm,
                         min_clear=LAYOUT.min_clearance["MAIN"])
        for got, want in zip(c.segments, paths.MAIN):
            self.assertEqual((got.direction, got.side, got.radius), (want.direction, want.side, want.radius))
            self.assertAlmostEqual(got.amount, want.amount, delta=0.01)

    def test_solver_keeps_close_in_its_clearance_band(self):
        lo, hi = LAYOUT.min_clearance["CLOSE"], LAYOUT.max_clearance["CLOSE"]
        c = solver.solve(A, B, O0, CAR, LAYOUT, side1="R", lead_in=10.0, min_clear=lo, max_clear=hi)
        self.assertTrue(lo <= c.clearance <= hi)

    def test_solver_reports_no_solution(self):
        self.assertIsNone(solver.solve(A, B, O0, CAR, LAYOUT, side1="R", lead_in=12.0, min_clear=10.0))


class ResolvedAlt(unittest.TestCase):
    def main_poses(self):
        return poses(A, paths.MAIN)

    def test_alt_has_ramp_straights_and_clearance(self):
        for s in (paths.ALT[0], paths.ALT[-1]):
            self.assertFalse(s.is_arc)
            self.assertGreaterEqual(s.length, CAR.ramp_cm)
        c = clear(A, paths.ALT, O0)
        self.assertGreaterEqual(c, LAYOUT.min_clearance["ALT"])
        self.assertAlmostEqual(c, 12.9, delta=0.2)                # user's check: ~12.9 cm
        self.assertLess(g.pose_error(g.end_pose(A, paths.ALT, OFF), B)[0], 0.5)

    def test_alt_looks_different_from_main(self):
        d = g.path_difference(poses(A, paths.ALT), self.main_poses())
        self.assertAlmostEqual(d, 5.0, delta=0.5)                 # user's check: ~5 cm
        # more different than the previous ALT (R35/R80, 49°) and the brief's
        old = g.segs("F 0 8 | F R R35 49 | F 0 0.59 | F L R80 49 | F 0 9.82")
        self.assertGreater(d, g.path_difference(poses(A, old), self.main_poses()) + 1.0)
        self.assertGreater(d, g.path_difference(poses(A, paths.ALT_BRIEF), self.main_poses()))

    def test_solver_finds_no_more_different_alt(self):
        ref = [p for p, _ in g.sample(A, paths.MAIN, OFF, 1.0)]
        c = solver.solve(A, B, O0, CAR, LAYOUT, side1="R", lead_in=CAR.ramp_cm,
                         min_clear=LAYOUT.min_clearance["ALT"], reference=ref)
        ours = g.path_difference([p for p, _ in g.sample(A, paths.ALT, OFF, 1.0)], ref)
        self.assertLess(c.difference - ours, 0.1)

    def test_path_difference(self):
        a = [g.Pose(x, 0, 0) for x in range(0, 101, 10)]
        b = [g.Pose(x, 3 if x == 50 else 0, 0) for x in range(0, 101, 10)]
        self.assertAlmostEqual(g.path_difference(a, b), 3)
        self.assertEqual(g.path_difference(a, a), 0)


class Event(unittest.TestCase):
    def stop(self, travel=LAYOUT.event_stop_travel_cm):
        return g.end_pose(A, paths.event_approach(travel), OFF)

    def test_backup_pose(self):
        b = g.end_pose(self.stop(), paths.EVENT_BACKUP, OFF)
        self.assertAlmostEqual(b.x, 33.8, delta=0.5)
        self.assertAlmostEqual(b.y, 67.7, delta=0.5)
        self.assertAlmostEqual(b.h, -15, places=6)

    def test_ultrasonic_confirms_on_time_obstacle(self):
        d = paths.ultrasonic_reading(self.stop(), LAYOUT, CAR, "O1")
        self.assertAlmostEqual(d, 22, delta=1.5)
        self.assertLess(d, LAYOUT.event_late_cm)
        # obstacle still at O0 (late): nothing within the late threshold
        late = paths.ultrasonic_reading(self.stop(), LAYOUT, CAR, "O0")
        self.assertTrue(late is None or late > LAYOUT.event_late_cm)

    def test_replan_reaches_c_with_clearance(self):
        b = g.end_pose(self.stop(), paths.EVENT_BACKUP, OFF)
        self.assertLess(g.pose_error(g.end_pose(b, paths.EVENT_REPLAN, OFF), C)[0], 0.5)
        self.assertAlmostEqual(clear(b, paths.EVENT_REPLAN, O1), 11.4, delta=0.3)

    def test_sensitivity_two_cm_moves_arrival_one_cm_sideways(self):
        arrive = {}
        for t in (21.5, 23.5, 25.5):
            b = g.end_pose(self.stop(t), paths.EVENT_BACKUP, OFF)
            arrive[t] = g.end_pose(b, paths.EVENT_REPLAN, OFF)
        for t in (21.5, 25.5):
            side = arrive[t].y - arrive[23.5].y      # car faces +x at C, so sideways is y
            self.assertAlmostEqual(abs(side), 1.0, delta=0.3)
        self.assertLess((arrive[21.5].y - arrive[23.5].y) * (arrive[25.5].y - arrive[23.5].y), 0)


class LHStop(unittest.TestCase):
    def test_expected_sensor_stop(self):
        d = paths.lh_lane_stop_cm(LAYOUT, CAR)
        self.assertAlmostEqual(d, 10, delta=1)
        lane, _ = paths.lh_lane_ac_planned(LAYOUT, CAR)
        p = g.end_pose(A, lane + [g.Segment("F", "0", 0, d)], OFF)
        self.assertAlmostEqual(p.x, 39.8, delta=0.5)
        self.assertAlmostEqual(p.y, 45, delta=0.1)

    def test_cap_when_obstacle_is_absent(self):
        far = replace(LAYOUT, o1=(140.0, 45.0))
        self.assertEqual(paths.lh_lane_stop_cm(far, CAR), LAYOUT.lh_detect_cap_cm)


class Planner(unittest.TestCase):
    def test_every_profile_passes_every_rule(self):
        for p in paths.PROFILE_KEYS:
            with self.subTest(profile=p):
                rep = check_profile(p, LAYOUT, CAR)
                self.assertEqual(rep.fails, [])

    def test_every_leg_ends_on_its_target(self):
        for p in paths.PROFILE_KEYS:
            for r in check_profile(p, LAYOUT, CAR).rows:
                if r["end"]:
                    self.assertLess(r["end"][0], LAYOUT.end_tolerance_cm, (p, r["name"]))
                    self.assertLess(r["end"][1], LAYOUT.end_tolerance_deg, (p, r["name"]))

    def test_clearance_thresholds(self):
        for p in paths.PROFILE_KEYS:
            for r in check_profile(p, LAYOUT, CAR).rows:
                self.assertGreaterEqual(r["clear"], r["need"], (p, r["name"], r["phase"]))
                self.assertGreaterEqual(r["edge"], r["margin"], (p, r["name"], r["phase"]))

    def test_profiles_follow_the_storyboards(self):
        names = {p: [(l.start, l.end) for l in paths.legs(p, LAYOUT, CAR)] for p in paths.PROFILE_KEYS}
        self.assertEqual(names["HH"], [("A", "B"), ("B", "A"), ("A", "C"), ("C", "A"), ("A", "D")])
        self.assertEqual(names["HL"], [("A", "D"), ("D", "A"), ("A", "B"), ("B", "A"), ("A", "C")])
        self.assertEqual(names["LH"], [("A", "B"), ("B", "A"), ("A", "A"), ("A", "D")])
        for p in paths.PROFILE_KEYS:     # one continuous take: each leg starts where the last ended
            ls = paths.legs(p, LAYOUT, CAR)
            for a, b in zip(ls, ls[1:]):
                self.assertEqual(a.end, b.start)

    def test_ll_is_a_stub(self):
        with self.assertRaises(NotImplementedError):
            paths.legs("LL", LAYOUT, CAR)

    def test_wider_calibrated_car_is_refused(self):
        rep = check_profile("HL", LAYOUT, replace(CAR, half_width=12.0))
        self.assertFalse(rep.ok)

    def test_larger_minimum_radius_is_refused(self):
        rep = check_profile("HH", LAYOUT, replace(CAR, min_radius=35.0))
        self.assertTrue(any("minimum radius" in f for f in rep.fails))

    def test_final_straight_tape_stop_needs_a_final_straight(self):
        with mock.patch.object(paths, "MAIN", paths.MAIN_BRIEF):
            rep = check_profile("HH", LAYOUT, CAR)
            self.assertTrue(any("tape stop at A" in f for f in rep.fails))
            tape = dict(LAYOUT.tape, A=TapeStop(mode="last_n"))
            rep = check_profile("HH", replace(LAYOUT, tape=tape), CAR)
            self.assertFalse(any("tape stop" in f for f in rep.fails))

    def test_car_model_reads_measured_values_only_if_their_check_passed(self):
        data = {"car": {"half_width": "10", "min_radius": 32, "unknown": 1}}
        self.assertEqual(car_model(data), CAR)                                    # no checks passed
        data["checks"] = {"footprint": True, "radius_table": True}
        car = car_model(data)
        self.assertEqual((car.half_width, car.min_radius, car.front), (10.0, 32.0, CAR.front))


if __name__ == "__main__":
    unittest.main()
