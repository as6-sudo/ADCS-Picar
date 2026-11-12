# obs_avoidance_profile_new — four obstacle-response behaviour profiles

Four PiCar-X behaviours that react differently to an obstacle straight ahead. Each one starts
facing the obstacle and drives straight at it. The car detects the obstacle with the fixed
front ultrasonic sensor. "Head" always means the **camera pan servo**. There's no vision and
no new dependencies: the code only uses `picarx` (the same library as `time_traversal`) and
the Python standard library.

| | A — Repetitive / reactive | B — Fast / decisive | C — Slow / cautious | D — Persistent / non-avoidant |
|---|---|---|---|---|
| Approach speed | 40 | highest calibrated safe speed (70 by default) | 25 | 45 |
| Stops at | 30 cm | 30 cm | 30 cm | 20 cm (deliberately closer) |
| Head scan | 3 rapid L-R sweeps, 600°/s, ±35° | 1 sweep, 220°/s | 1 sweep, 50°/s, 0.6 s dwell | none (head fixed forward) |
| Scan/reverse cycles | 3 × (scan → reverse 0.35 s → stop) | 0 | scan → reverse 0.8 s → scan | — |
| Pauses | 0.15 s | 0.08 s | 0.9 s (visible hesitation) | 0.3 s stalls |
| Exit | forward right arc ≈90°, then straight out to the right | quick two-leg right U-turn (≈155°) at speed 70, then out | the same U-turn at speed 30 with a pause between the legs, then out | **no exit**: 6 × recoil + lunge "attempts", then holds still |
| Sim run time | ≈10 s | ≈5.5 s | ≈21 s | ≈8 s |

## State sequences

* **A:** APPROACH → STOP → [SCAN_i → REVERSE_i → STOP_i] × 3 → EXIT_TURN_RIGHT → EXIT_STRAIGHT
* **B:** APPROACH → STOP → SCAN → DECIDE → UTURN_REVERSE_LEFT_WHEELS → UTURN_PAUSE → UTURN_FORWARD_RIGHT_WHEELS → EXIT_STRAIGHT
* **C:** APPROACH → STOP → SCAN_1 → HESITATE_1 → REVERSE_1 → STOP_1 → SCAN_2 → HESITATE_2 → UTURN_REVERSE_LEFT_WHEELS → UTURN_PAUSE → UTURN_FORWARD_RIGHT_WHEELS → EXIT_STRAIGHT
* **D:** APPROACH → STALL → [RECOIL_i → ATTEMPT_i → STALL_i] × 6 → HOLD

Each state finishes (and stops the motors) before the next one starts. Every run begins and
ends with the motors stopped and the steering, pan and tilt servos centred.

## Steering sequences for the right-hand exits

The PiCar-X steers its front wheels like a car (+ = right, max 30°). It can't spin in place,
and its minimum turning radius is about 25–35 cm.

* **A (right turn):** A has already backed off three times (to about 60 cm), so there is room
  for a forward arc. Wheels go to +30 and it drives forward for 1.05 s, turning about 90°
  clockwise. Then wheels go to 0 and it drives straight off to the right.
* **B/C (right U-turn):** they stop only 30 cm from the obstacle, which is too close for a
  forward U-turn, so they turn in two legs:
  1. wheels −30 (left), **reverse**: the nose swings right by about 85° while the car backs away;
  2. wheels +30 (right), **forward**: it keeps turning clockwise to about 150–155°;
  3. wheels 0, forward: it leaves facing back and to the right of where it started.

  B does this at speed 70 with almost no pauses. C does it at speed 30 with a 0.9 s pause
  between the legs.
* **D:** the steering never moves.

The turns are timed, because there's no heading sensor, so the actual angles depend on the
floor and the battery. Tune `turn_s`, `uturn_reverse_s` and `uturn_forward_s` in `params.py`.
At a reduced `--speed-scale` the turns will be smaller.

## Profile D's safety

D never actually touches the obstacle. It stops at 20 cm, then repeats: back off for 0.25 s,
then lunge forward for 0.22 s at the same speed. The lunge is shorter, so the car drifts
slightly *away* over time. Every lunge also stops if the obstacle is confirmed closer than
`HARD_MIN_CM` (12 cm). In the simulation its closest approach is about 13 cm.

## Safety and sensing

* **Bad readings.** Values of 0, −2, < 2 cm, > 400 cm and NaN are ignored. A `-1` (picarx's
  "10 timeouts in a row") counts as nothing ahead, the same rule `time_traversal.core` uses.
  An obstacle only counts after **3 consecutive** close readings, so single false echoes from
  motor noise don't stop the car.
* **Hard stop.** Any profile stops immediately on 2 consecutive readings under 12 cm.
* **Approach timeout.** If no obstacle is found within 8 s, the car stops and the profile aborts.
* **Exit legs.** Forward exit legs stop early if something is within 25 cm. Reversing is
  unguarded (there's no rear sensor), so keep about 50 cm clear behind the car.
* **Emergency stop.** Press **Enter** (or Ctrl+C, or send SIGTERM). The motors stop and the
  servos centre on any error, abort or interrupt.
* **Speed gate.** Until every `calibrate.py` check passes, hardware runs are capped at 50 %
  speed and profile D is refused.

## How to run

On the laptop (no hardware):

```bash
python obs_avoidance_profile_new/run.py B --sim
python -m unittest discover -s obs_avoidance_profile_new/tests -t .
```

On the car (from the repo root, e.g. `~/picar-x`):

```bash
python3 obs_avoidance_profile_new/calibrate.py        # api, ultrasonic, motors, camera_pan, steering, stop_distance
python3 obs_avoidance_profile_new/run.py A --speed-scale 0.4   # each profile at reduced speed first
python3 obs_avoidance_profile_new/run.py A            # full speed once calibration has passed
python3 -m unittest discover -s obs_avoidance_profile_new/tests -t .   # also checks the installed picarx API
```

`calibration.json` is specific to each car, so it is git-ignored.

## Files

| File | Purpose |
|---|---|
| `params.py` | every tunable: speeds, thresholds, pan angle/speed, sweeps, cycles, pauses, turn speeds/durations, exit type |
| `robot.py` | blocking primitives (approach, move, pan_to, scan, centre), reading validation, e-stop |
| `profiles.py` | profiles A–D as ordered state lists |
| `runner.py` | runs the states and always shuts down safely |
| `run.py` | CLI: pick one profile; `--sim` for no hardware |
| `calibrate.py` | interactive hardware checks; writes `calibration.json` |
| `sim.py` | car simulator that works like a Picarx (kinematics, box obstacle, noisy sensor) |
| `tests/` | 19 unittest tests (safety, behaviour, compatibility) |

It reuses `time_traversal.core` (`clamp`, `MAX_STEER`, `NO_READING`) and calls the same Picarx
methods. `time_traversal` itself is unchanged.
