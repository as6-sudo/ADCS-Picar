"""Executor, simulator, profile and safety tests (laptop, no hardware):

    python -m unittest discover -s trajectory_profiles/tests -t .

On the Raspberry Pi the API test also checks the installed picarx.
"""
import io
import os
import sys
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from trajectory_profiles import calibration, geometry as g, paths, profiles, repeat, run  # noqa: E402
from trajectory_profiles.calibration import DEFAULT_CAR as CAR  # noqa: E402
from trajectory_profiles.executor import EStop, Executor, HardStop, Ranger, SensorStop, TapeSpec  # noqa: E402
from trajectory_profiles.layout import DEFAULT as LAYOUT  # noqa: E402
from trajectory_profiles.motion import Motion  # noqa: E402
from trajectory_profiles.sim import ArenaSim  # noqa: E402

M = Motion(calibration.tables())
QUIET = dict(log=lambda *a: None)


def make(profile="HH", start=None, strips=True, seed=0, estop=None):
    sim = ArenaSim(LAYOUT, CAR, start=start, seed=seed,
                   strips=list(profiles.strips(profile, LAYOUT, CAR, M).values()) if strips else [])
    recs = []
    ex = Executor(sim, M, clock=sim, estop=estop or EStop(), ranger=Ranger(sim, sim), ramp_cm=CAR.ramp_cm,
                  record=lambda **k: recs.append((sim.t, k)), **QUIET)
    return sim, ex, recs


def take(profile, only_leg=None, **kw):
    start = LAYOUT.target(paths.legs(profile, LAYOUT, CAR)[only_leg - 1].start) if only_leg else None
    sim, ex, recs = make(profile, start=start, **kw)
    t = profiles.Take(profile, ex, LAYOUT, CAR, M, sim=sim, cue_out=lambda: None, **QUIET)
    ok = t.run(only_leg=only_leg)
    return ok, sim, t, recs


def names(t):
    return [e[1] for e in t.events]


def at_rest_and_centred(test, sim):
    test.assertEqual(sim.motor_history[-1][1], "stop")
    test.assertEqual((sim.dir_target, sim.pan, sim.tilt), (0, 0, 0))


class MotionModel(unittest.TestCase):
    def test_radius_table_interpolation_and_sides(self):
        self.assertAlmostEqual(M.steer_for("R", 30), 30)
        self.assertAlmostEqual(M.steer_for("L", 30), -30)       # picarx: - steers left
        self.assertEqual(M.steer_for("0", 0), 0)
        a40, a85 = M.steer_for("R", 40), M.steer_for("R", 85)
        self.assertTrue(30 > a40 > a85 > 0)

    def test_inner_wheel_slows_arcs(self):
        self.assertLess(M.ground_speed(60, 30), M.ground_speed(60, 0))
        self.assertAlmostEqual(M.ground_speed(60, 10), M.ground_speed(60, -10))

    def test_command_for_hits_the_speed(self):
        for v in (20, 35, 50):
            for steer in (0, 12, 30):
                self.assertAlmostEqual(M.ground_speed(M.command_for(v, steer), steer), v, delta=1.0)

    def test_minimum_speed(self):
        self.assertEqual(M.command_for(5), 1)                    # cannot go slower than PWM 50 %
        self.assertGreater(M.straight_speed(1), 10)


class Executor_(unittest.TestCase):
    def test_no_speed_change_inside_an_arc(self):
        sim, ex, _ = make()
        for s in (paths.MAIN, paths.ALT, g.retrace(paths.CLOSE)):
            pieces = ex._pieces(s, 50.0, (0.4, 0.6, 0.8, 1.0))
            for i, seg in enumerate(s):
                cmds = {p.cmd for p in pieces if p.seg == i}
                if seg.is_arc:
                    self.assertEqual(len(cmds), 1, str(seg))
            self.assertGreater(len({p.cmd for p in pieces if p.seg == 0}), 1)   # the ramp is on the straight

    def test_segment_lands_where_planned(self):
        for text in ("F 0 40", "F R R30 30", "B L R30 30", "F L R85 49"):
            for v in (20, 50):
                sim, ex, _ = make(start=g.Pose(75, 75, 0), strips=False)
                sim.place_obstacle((500, 500))
                ex.run([g.seg(text)], leg="t", speed=v, rolling=False)
                sim.sleep(0.4)
                want = g.end_pose(g.Pose(75, 75, 0), [g.seg(text)], CAR.rear_to_centre)
                self.assertLess(g.pose_error(sim.pose(), want)[0], 0.6, (text, v))

    def test_rolling_main_and_retrace(self):
        for v, ramps in ((50, (0.4, 0.6, 0.8, 1.0)), (25, (0.7, 0.8, 0.9, 1.0))):
            sim, ex, _ = make(strips=False)
            sim.place_obstacle((500, 500))
            res = ex.run(paths.MAIN, leg="t", speed=v, ramp_fracs=ramps)
            sim.sleep(0.4)
            self.assertLess(g.pose_error(sim.pose(), LAYOUT.target("B"))[0], 3.0)
            ex.retrace(res.entries, leg="t", speed=v, ramp_fracs=ramps)
            sim.sleep(0.4)
            self.assertLess(g.pose_error(sim.pose(), LAYOUT.target("A"))[0], 1.5)

    def test_log_records_actual_time_when_cut_short(self):
        sim, ex, _ = make(start=g.Pose(30, 45, 0), strips=False)       # obstacle face 42.5 cm ahead
        sim.place_obstacle(LAYOUT.o1)
        res = ex.run([g.seg("F 0 40")], leg="t", speed=35, rolling=False, sensor_stop=SensorStop(20.0))
        sim.sleep(0.4)
        e = res.entries[-1]
        self.assertEqual(e.event, "OBSTACLE_STOP")
        self.assertLess(e.actual_s, e.planned_s)
        self.assertLess(e.driven.length, 40)
        # the logged distance matches what the car really did (within 1 cm)
        self.assertAlmostEqual(sim.pose().x - 30, e.driven.length, delta=1.0)

    def test_hard_minimum_stops_forward_moves(self):
        sim, ex, _ = make(start=g.Pose(40, 75, 0), strips=False)          # O0 straight ahead
        with self.assertRaises(HardStop):
            ex.run([g.seg("F 0 60")], leg="t", speed=35, rolling=False)
        self.assertFalse(sim.collided)
        self.assertEqual(sim.motor_history[-1][1], "stop")

    def test_reverse_moves_are_unguarded(self):
        sim, ex, _ = make(start=g.Pose(40, 75, 0), strips=False)
        ex.run([g.seg("B 0 10")], leg="t", speed=35, rolling=False)        # obstacle ahead: no stop
        sim.sleep(0.4)
        self.assertAlmostEqual(sim.pose().x, 30, delta=0.6)

    def test_tape_stop_and_tape_missed(self):
        start = g.Pose(95, 35, 0)
        tape = TapeSpec()
        sim, ex, _ = make("LH", start=start)
        res = ex.run([g.seg("F 0 30")], leg="t", speed=35, rolling=False, tape=tape)
        sim.sleep(0.4)
        self.assertEqual(res.events[-1][0], "TAPE_STOP")
        self.assertAlmostEqual(sim.pose().x, 125, delta=1.0)
        sim, ex, _ = make("LH", start=start, strips=False)
        res = ex.run([g.seg("F 0 30")], leg="t", speed=35, rolling=False, tape=tape)
        sim.sleep(0.4)
        self.assertEqual(res.events[-1][0], "TAPE_MISSED")
        self.assertAlmostEqual(sim.pose().x, 125 + tape.miss_margin_cm, delta=1.0)

    def test_long_final_straight_ignores_the_strip_it_starts_on(self):
        ok, sim, t, _ = take("HH", only_leg=4)        # C -> A: starts on C's strip
        self.assertTrue(ok)
        self.assertAlmostEqual(sim.pose().x, 20, delta=1.5)


class Safety(unittest.TestCase):
    def test_estop_at_any_moment_leaves_car_stopped_and_centred(self):
        for profile in ("HH", "HL", "LH"):
            for at in (0.5, 2.0, 7.5, 12.0):
                with self.subTest(profile=profile, at=at):
                    estop = EStop()
                    sim, ex, _ = make(profile, estop=estop)
                    sleep = sim.sleep

                    def trip(s, sleep=sleep, sim=sim, estop=estop, at=at):
                        sleep(s)
                        if sim.t >= at:
                            estop.trigger("test")
                    sim.sleep = trip
                    t = profiles.Take(profile, ex, LAYOUT, CAR, M, sim=sim, cue_out=lambda: None, **QUIET)
                    self.assertFalse(t.run())
                    self.assertIn("ESTOP", names(t))
                    at_rest_and_centred(self, sim)
                    self.assertLess(sim.t, at + 0.5)

    def test_shutdown_never_raises(self):
        sim, ex, _ = make()

        def boom(*a):
            raise RuntimeError("i2c error")
        for m in ("stop", "set_dir_servo_angle", "set_cam_pan_angle", "set_cam_tilt_angle"):
            setattr(sim, m, boom)
        ex.safe_shutdown()                    # must not raise

    def test_error_mid_take_still_stops_and_centres(self):
        sim, ex, _ = make()
        calls = {"n": 0}
        real = sim.get_distance

        def flaky():
            calls["n"] += 1
            if calls["n"] == 50:
                raise RuntimeError("sensor fell off")
            return real()
        sim.get_distance = flaky
        t = profiles.Take("HH", ex, LAYOUT, CAR, M, sim=sim, cue_out=lambda: None, **QUIET)
        with self.assertRaises(RuntimeError):
            t.run()
        at_rest_and_centred(self, sim)


class Takes(unittest.TestCase):
    def test_full_takes_in_the_simulator(self):
        for profile in ("HH", "HL", "LH"):
            with self.subTest(profile=profile):
                ok, sim, t, _ = take(profile)
                self.assertTrue(ok)
                last = paths.legs(profile, LAYOUT, CAR)[-1].end
                self.assertLess(g.pose_error(sim.pose(), LAYOUT.target(last))[0], 2.0)
                self.assertFalse(sim.collided)
                self.assertGreater(sim.min_obstacle_clearance, LAYOUT.min_clearance["CLOSE"])
                self.assertGreater(sim.min_edge_clearance, LAYOUT.lane_edge_margin - 0.5)
                for bad in ("OBSTACLE_LATE", "TAPE_MISSED", "NO_OBSTACLE", "ABORT", "ESTOP"):
                    self.assertNotIn(bad, names(t))
                at_rest_and_centred(self, sim)

    def test_obstacle_late(self):
        sim, ex, _ = make("HH")
        sim.start_pull = lambda delay=0.0: None          # nobody pulls the line
        t = profiles.Take("HH", ex, LAYOUT, CAR, M, sim=sim, cue_out=lambda: None, **QUIET)
        self.assertFalse(t.run(only_leg=3))
        self.assertIn("OBSTACLE_LATE", names(t))
        self.assertIn("RETAKE", names(t))
        self.assertNotIn("ABORT", names(t))              # ended before the re-plan, not by the hard stop
        # it never drove the back-up or the re-plan: it is still where the approach stopped
        stop = g.end_pose(LAYOUT.target("A"), paths.event_geometry("HH", LAYOUT)[0], CAR.rear_to_centre)
        self.assertLess(g.pose_error(sim.pose(), stop)[0], 2.0)
        self.assertGreater(sim.min_obstacle_clearance, 15.0)
        at_rest_and_centred(self, sim)

    def test_lh_sensor_stop_position(self):
        ok, sim, t, recs = take("LH", only_leg=3)
        self.assertTrue(ok)
        self.assertIn("OBSTACLE_STOP", names(t))
        stop = [k for _, k in recs if k.get("event") == "OBSTACLE_STOP" and k["kind"] == "segment"][0]
        into_lane = float(stop["driven"].split()[-1])
        # reading drops below 20 cm ~9.5 cm into the lane; 3 confirmations + the coast
        # add a few cm, and it must happen before the 15 cm cap
        self.assertTrue(9.0 <= into_lane < LAYOUT.lh_detect_cap_cm, into_lane)
        self.assertAlmostEqual(sim.pose().x, 20, delta=1.5)   # retraced to A

    def test_lh_cap_when_nothing_is_there(self):
        sim, ex, recs = make("LH")
        sim.start_pull = lambda delay=0.0: None
        sim.place_obstacle((140.0, 140.0))
        t = profiles.Take("LH", ex, LAYOUT, CAR, M, sim=sim, cue_out=lambda: None, **QUIET)
        self.assertTrue(t.run(only_leg=3))
        self.assertIn("NO_OBSTACLE", names(t))
        seg = [k for _, k in recs if k.get("event") == "NO_OBSTACLE" and k["kind"] == "segment"][0]
        self.assertAlmostEqual(float(seg["driven"].split()[-1]), LAYOUT.lh_detect_cap_cm, delta=1.5)
        self.assertAlmostEqual(sim.pose().x, 20, delta=1.5)   # still retraced to A

    def test_cue_timing(self):
        reach = LAYOUT.operator_reaction_s + 30 / LAYOUT.obstacle_slide_cm_s     # obstacle at O1
        for profile, leg, expect, tol in (("HH", 3, 1.0, 0.15), ("HL", 5, 1.4, 0.1)):
            ok, sim, t, recs = take(profile, only_leg=leg)
            self.assertTrue(ok, profile)
            cue = [s for s, k in recs if k.get("event") == "CUE"][0]
            last = len(paths.event_geometry(profile, LAYOUT)[0]) - 1     # the approach's last segment
            stop = [s for s, k in recs if k["kind"] == "segment" and k["index"] == last][0]
            self.assertAlmostEqual(stop - cue, expect, delta=tol, msg=profile)
            self.assertLess(cue + reach, stop, profile)                 # obstacle there before the stop
        self.assertAlmostEqual(LAYOUT.cue["HL"], 0.3)                   # the HL delay lives in the config
        # LH: cue when the first pivot is done; obstacle in place before the lower lane
        ok, sim, t, recs = take("LH", only_leg=3)
        cue = [s for s, k in recs if k.get("event") == "CUE"][0]
        pivot_done = [s for s, k in recs if k["kind"] == "segment" and k["index"] == 3][0]
        lane = [s for s, k in recs if k["kind"] == "segment" and k["index"] == 7][0]
        self.assertAlmostEqual(cue, pivot_done, delta=0.05)
        reach = LAYOUT.operator_reaction_s + 30 / LAYOUT.obstacle_slide_cm_s
        self.assertLess(cue + reach, lane)

    def test_ll_is_a_stub(self):
        sim, ex, _ = make()
        with self.assertRaises(NotImplementedError):
            profiles.Take("LL", ex, LAYOUT, CAR, M)
        with self.assertRaises(NotImplementedError):
            run.main(["LL", "--dry-run"])


class RunCli(unittest.TestCase):
    def test_dry_run_and_sim(self):
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(run.main(["HL", "--dry-run"]), 0)
        self.assertIn("Tape strips", out.getvalue())

    def test_refuses_when_plan_fails(self):
        import json
        import tempfile
        p = os.path.join(tempfile.mkdtemp(), "calibration.json")
        with open(p, "w") as f:      # a measured car too wide for CLOSE
            json.dump({"checks": {"footprint": True}, "car": {"half_width": 13.0}}, f)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(run.main(["HL", "--dry-run", "--calibration", p]), 2)


class Repeat(unittest.TestCase):
    T = g.Pose(125, 35, 0)

    def test_tight_spread_tapes_the_x(self):
        r = repeat.analyse([(125, 35), (126, 35.5), (124.5, 34.5)], self.T)
        self.assertTrue(r["recommendation"].startswith("tape the X"))

    def test_along_spread_enables_tape_stop(self):
        r = repeat.analyse([(120, 35), (126, 35.5), (129, 34.5)], self.T)
        self.assertIn("tape stop", r["recommendation"])
        self.assertAlmostEqual(r["along_spread"], 9)

    def test_sideways_spread_tunes_the_curve(self):
        r = repeat.analyse([(125, 30), (126, 35.5), (124.5, 40)], self.T)
        self.assertIn("tune the curve", r["recommendation"])
        self.assertAlmostEqual(r["side_spread"], 10)

    def test_parse_landing(self):
        self.assertEqual(repeat.parse_landing("124 36", self.T), (124.0, 36.0))
        self.assertEqual(repeat.parse_landing("d -1 2", self.T), (124.0, 37.0))


class PicarxApi(unittest.TestCase):
    def test_installed_picarx_matches_the_brief(self):
        try:
            from picarx import Picarx
        except Exception:
            self.skipTest("picarx not installed (laptop)")
        for m in ("forward", "backward", "stop", "set_dir_servo_angle", "set_cam_pan_angle",
                  "set_cam_tilt_angle", "get_distance", "get_grayscale_data"):
            self.assertTrue(callable(getattr(Picarx, m, None)), m)
        self.assertEqual((Picarx.DIR_MIN, Picarx.DIR_MAX), (-30, 30))
        self.assertGreaterEqual(Picarx.CAM_PAN_MAX, 60)


if __name__ == "__main__":
    unittest.main()
