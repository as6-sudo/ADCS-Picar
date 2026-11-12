"""Executes a profile's states in order, and always leaves the car stopped."""
from .profiles import build_states
from .robot import EmergencyStop, ProfileAbort


def run_profile(robot, params, max_safe_speed, log=print):
    """Run one profile. Returns True if every state completed.

    Emergency stops and aborts return False. Any other exception (including
    KeyboardInterrupt) is re-raised, but only after the motors are stopped
    and the servos centred."""
    states = build_states(params, max_safe_speed)
    t0 = robot.clock.now()
    ok = False
    try:
        robot.center()
        for i, st in enumerate(states, 1):
            robot.estop.check()
            log("[%s] %6.2fs  %2d/%d  %s" % (params.key, robot.clock.now() - t0, i, len(states), st.name))
            robot.record("state", name=st.name)
            st.action(robot)
        ok = True
    except EmergencyStop as e:
        log("[%s] EMERGENCY STOP: %s" % (params.key, e))
    except ProfileAbort as e:
        log("[%s] aborted: %s" % (params.key, e))
    finally:
        robot.safe_shutdown()
    log("[%s] %s %s after %.2fs (%d invalid ultrasonic readings ignored)"
        % (params.key, params.name, "finished" if ok else "stopped",
           robot.clock.now() - t0, robot.invalid_readings))
    return ok
