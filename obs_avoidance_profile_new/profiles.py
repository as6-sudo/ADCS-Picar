"""The four profiles as explicit, ordered state sequences.

`build_states(params, max_safe_speed)` returns a list of State(name, action);
the runner executes them one after another, each to completion.

Steering sequences used for the right-side exits (PiCar-X is Ackermann-
steered, so it cannot spin in place; minimum turning radius is roughly
25-35 cm at 30 deg of steering):

  A  "right_turn":  wheels +30 (right), drive FORWARD -> the car arcs
                    ~90 deg clockwise; wheels 0, drive forward out to the
                    right. A has backed off 3x before this, so the arc has
                    room in front of it.
  B/C "right_uturn" (a two-leg turn, needed because the car stops only
                    ~30 cm from the obstacle, less than a forward U-turn needs):
                    1. wheels -30 (LEFT), drive BACKWARD -> the nose swings
                       to the right (clockwise) by ~90 deg while the car backs
                       away from the obstacle;
                    2. wheels +30 (RIGHT), drive FORWARD -> it keeps turning
                       clockwise, ending ~150 deg from the start (facing back
                       and to the right);
                    3. wheels 0, drive forward out of the scene to the right.
                    B does this at speed 70 with almost no pauses, C at speed
                    30 with a hesitation pause between the legs.
"""
from collections import namedtuple

from .params import APPROACH_TIMEOUT_S, EXIT_GUARD_CM, HARD_MIN_CM

State = namedtuple("State", "name action")


def _approach(P, speed):
    return State("APPROACH", lambda r: r.approach(speed, P.detect_cm, APPROACH_TIMEOUT_S))


def _pause(name, s):
    return State(name, lambda r: r.wait(s))


def _scan(name, P):
    return State(name, lambda r: r.scan(P.pan_angle, P.pan_speed_dps, P.scan_sweeps, P.pan_hold_s))


def _reverse(name, P):
    return State(name, lambda r: r.move(-1, P.reverse_speed, P.reverse_s, steer=0))


def _forward(name, speed, seconds, steer=0, guard=EXIT_GUARD_CM):
    return State(name, lambda r: r.move(+1, speed, seconds, steer=steer, guard_cm=guard))


def _exit_right_turn(P):
    return [
        _forward("EXIT_TURN_RIGHT", P.turn_speed, P.turn_s, steer=+P.turn_steer),
        _forward("EXIT_STRAIGHT", P.exit_speed, P.exit_s),
    ]


def _exit_right_uturn(P):
    return [
        State("UTURN_REVERSE_LEFT_WHEELS",
              lambda r: r.move(-1, P.turn_speed, P.uturn_reverse_s, steer=-P.turn_steer)),
        _pause("UTURN_PAUSE", P.pause_s),
        _forward("UTURN_FORWARD_RIGHT_WHEELS", P.turn_speed, P.uturn_forward_s, steer=+P.turn_steer),
        _forward("EXIT_STRAIGHT", P.exit_speed, P.exit_s),
    ]


def profile_a(P, speed):
    """approach -> stop -> [fast scan x3 sweeps -> reverse] x3 -> right turn -> exit."""
    s = [_approach(P, speed), _pause("STOP", P.pause_s)]
    for i in range(1, P.scan_cycles + 1):
        s += [_scan("SCAN_%d" % i, P), _reverse("REVERSE_%d" % i, P), _pause("STOP_%d" % i, P.pause_s)]
    return s + _exit_right_turn(P)


def profile_b(P, speed):
    """fast approach -> stop -> one quick scan -> quick right U-turn -> exit."""
    return [_approach(P, speed), _pause("STOP", P.pause_s), _scan("SCAN", P),
            _pause("DECIDE", P.pause_s)] + _exit_right_uturn(P)


def profile_c(P, speed):
    """slow approach -> stop -> slow scan -> reverse -> stop -> slow scan -> slow U-turn -> exit."""
    s = [_approach(P, speed), _pause("STOP", P.pause_s)]
    for i in range(1, P.scan_cycles + 1):
        s += [_scan("SCAN_%d" % i, P), _pause("HESITATE_%d" % i, P.pause_s)]
        if i < P.scan_cycles:
            s += [_reverse("REVERSE_%d" % i, P), _pause("STOP_%d" % i, P.pause_s)]
    return s + _exit_right_uturn(P)


def profile_d(P, speed):
    """approach -> stop at safe minimum -> repeated recoil+lunge attempts -> hold. No scan, no exit.

    The "attempts" are a short back-off followed by an equal-or-shorter
    forward lunge, so the car looks like it keeps pushing at the obstacle
    while its net position drifts slightly backwards. Every lunge is guarded
    by HARD_MIN_CM, so it can never reach the obstacle."""
    s = [_approach(P, speed), _pause("STALL", P.pause_s)]
    for i in range(1, P.attempts + 1):
        s += [State("RECOIL_%d" % i, lambda r: r.move(-1, P.attempt_speed, P.recoil_s)),
              _forward("ATTEMPT_%d" % i, P.attempt_speed, P.lunge_s, guard=HARD_MIN_CM),
              _pause("STALL_%d" % i, P.pause_s)]
    return s + [State("HOLD", lambda r: r.stop())]


BUILDERS = {"A": profile_a, "B": profile_b, "C": profile_c, "D": profile_d}


def build_states(P, max_safe_speed):
    speed = P.forward_speed or max_safe_speed
    return BUILDERS[P.key](P, speed)
