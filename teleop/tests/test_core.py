"""TeleopCore against the arena simulator (no hardware, simulated time).

    python -m unittest discover -s teleop/tests -t .
"""
import math
import unittest

from trajectory_profiles import calibration
from trajectory_profiles.executor import Ranger
from trajectory_profiles.layout import DEFAULT as LAYOUT
from trajectory_profiles.motion import Motion
from trajectory_profiles.sim import ArenaSim

from teleop.core import WATCHDOG_S, TeleopCore

DT = 0.02


def make(profile="HH", obstacle=LAYOUT.o1):
    res = calibration.Resolved(None)
    sim = ArenaSim(LAYOUT, res.car, tables=res.tables, invalid_rate=0.0, spike_rate=0.0)
    sim.place_obstacle(obstacle)
    core = TeleopCore(sim, Motion(res.tables), sim, Ranger(sim, sim), res.car, style=profile, seed=1)
    return sim, core


def run(sim, core, seconds, held=(), send=True):
    end = sim.t + seconds
    while sim.t < end:
        if send:
            core.keys(held)
        sim.sleep(DT)
        core.tick()


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class TestDriving(unittest.TestCase):
    def test_hold_to_drive_and_release_stops(self):
        sim, core = make("HH")
        x0 = sim.pose().x
        run(sim, core, 1.0, {"w"})
        self.assertGreater(sim.pose().x - x0, 25)
        self.assertEqual(core.out[0], 1)
        run(sim, core, 0.6)
        self.assertEqual(core.out[0], 0)
        self.assertAlmostEqual(sim.v, 0.0, places=3)

    def test_watchdog_stops_when_input_stops(self):
        sim, core = make("HH")
        run(sim, core, 0.5, {"w"})
        run(sim, core, WATCHDOG_S + 0.1, send=False)
        self.assertEqual(core.out[0], 0)
        self.assertEqual(core.status()["state"], "NO INPUT")
        run(sim, core, 0.2, {"w"})                  # input back: drives again
        self.assertEqual(core.out[0], 1)

    def test_space_stops_at_once_even_in_ll(self):
        sim, core = make("LL")
        run(sim, core, 1.0, {"w"})
        core.command("stop")
        run(sim, core, DT)
        self.assertEqual(core.out[0], 0)

    def test_ll_keeps_rolling_after_release(self):
        sim, core = make("LL")
        run(sim, core, 1.5, {"w"})
        run(sim, core, 0.8)
        self.assertEqual(core.out[0], 1, "LL should still be rolling 0.8 s after release")
        run(sim, core, 1.5)
        self.assertEqual(core.out[0], 0)

    def test_hl_ramps_slower_than_hh(self):
        def reach(profile):
            sim, core = make(profile)
            t = 0.0
            while core.v < core.cruise() - 0.5:
                run(sim, core, DT, {"w"})
                t += DT
            return t
        self.assertGreater(reach("HL"), 3 * reach("HH"))

    def test_lh_is_instant_and_pauses(self):
        sim, core = make("LH")
        run(sim, core, DT, {"w"})
        self.assertAlmostEqual(core.v, core.cruise())
        run(sim, core, 1.0, {"w"})
        run(sim, core, DT)
        self.assertEqual(core.out[0], 0)
        run(sim, core, 1.0, {"w"})                  # within the 2 s pause: stays put
        self.assertEqual(core.out[0], 0)
        run(sim, core, 1.2, {"w"})
        self.assertEqual(core.out[0], 1)

    def test_lh_pivot_turns_90(self):
        sim, core = make("LH")
        h0 = sim.pose().h
        run(sim, core, DT, {"d"})
        self.assertEqual(core.macro_name, "pivot right")
        run(sim, core, 6.0)
        self.assertIsNone(core.macro)
        self.assertAlmostEqual(wrap(sim.pose().h - h0), -90.0, delta=10.0)

    def test_profile_change_only_when_stopped(self):
        sim, core = make("HH")
        run(sim, core, 0.5, {"w"})
        core.command("profile", "LL")
        self.assertEqual(core.style.key, "HH")
        run(sim, core, 0.5)
        core.command("profile", "LL")
        self.assertEqual(core.style.key, "LL")


class TestSafety(unittest.TestCase):
    def test_hard_stop_before_the_obstacle(self):
        sim, core = make("HH", obstacle=LAYOUT.o0)       # O0 is straight ahead of A
        run(sim, core, 4.0, {"w"})
        self.assertFalse(sim.collided)
        self.assertEqual(core.out[0], 0)
        self.assertTrue(core.blocked)
        run(sim, core, 0.2)                              # release W: unblocked
        self.assertFalse(core.blocked)

    def test_reverse_still_allowed_when_blocked(self):
        sim, core = make("HH", obstacle=LAYOUT.o0)
        run(sim, core, 4.0, {"w"})
        x = sim.pose().x
        run(sim, core, 0.5, {"s"})
        self.assertLess(sim.pose().x, x - 5)


class TestMacros(unittest.TestCase):
    def test_retrace_returns_to_the_mark(self):
        for profile in ("HH", "HL", "LH"):
            with self.subTest(profile=profile):
                sim, core = make(profile)
                p0 = sim.pose()
                core.command("mark")
                run(sim, core, 1.0, {"w"})
                run(sim, core, 0.8, {"w", "d"} if profile != "LH" else {"w"})
                run(sim, core, 2.5)
                core.command("retrace")
                run(sim, core, 8.0)
                self.assertIsNone(core.macro)
                p = sim.pose()
                self.assertLess(math.hypot(p.x - p0.x, p.y - p0.y), 6.0, profile)
                self.assertLess(abs(wrap(p.h - p0.h)), 8.0, profile)

    def test_hh_surprise_backs_up(self):
        sim, core = make("HH")
        run(sim, core, 0.8, {"w"})
        x = sim.pose().x
        core.command("surprise")
        run(sim, core, 4.0)
        self.assertIsNone(core.macro)
        self.assertLess(sim.pose().x, x)
        self.assertEqual(core.out[0], 0)

    def test_space_cancels_a_macro(self):
        sim, core = make("LH")
        run(sim, core, DT, {"d"})
        run(sim, core, 0.3)
        core.command("stop")
        run(sim, core, DT)
        self.assertIsNone(core.macro)
        self.assertEqual(core.out[0], 0)

    def test_wiggle_ends_centred(self):
        sim, core = make("HH")
        core.command("wiggle")
        run(sim, core, 3.0)
        self.assertIsNone(core.macro)
        self.assertEqual(core.out[2], 0.0)
        self.assertEqual(core.out[0], 0)


if __name__ == "__main__":
    unittest.main()
