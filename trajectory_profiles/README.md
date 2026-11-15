# trajectory_profiles: scripted motion-profile takes (LH, HH, HL)

Four short videos of the PiCar-X doing the same task with different motion
profiles. Every behaviour is **scripted and open-loop**: the car only has to
*look* autonomous. Robustness comes from a fixed floor layout, careful
calibration, tape stops at the targets and a few ultrasonic checks.

| Profile | Legs (one continuous take) | Character |
|---|---|---|
| **HH** | A→B MAIN · B→A MAIN retraced · A→C obstacle event · C→A straight reverse · A→D MAIN mirrored | fast (50 cm/s), short ramps, looks at the target before legs 1 and 5, happy wiggle at D |
| **HL** | A→D MAIN mirrored · D→A ALT mirrored, retraced · A→B MAIN · B→A CLOSE retraced · A→C obstacle event | slow (25 cm/s), long ramps, looks at the obstacle after the surprise, happy wiggle at C |
| **LH** | A→B lane · B→A lane retraced · A→C lane → obstacle stop → retrace to A · A→D lane mirrored | constant 35 cm/s, instant starts/stops, 3-step pivots, 2.0 s pause at each target |
| LL | not implemented (raises `NotImplementedError`) | |

`time_traversal/` and `obs_avoidance_profile_new/` are unchanged; this package
reuses their e-stop, reading validation, hard-minimum constants and `SimCar`.

## Files

| File | Purpose |
|---|---|
| `layout.py` | arena, targets A–D, obstacle O0/O1, clearance rules, LH/event constants, tape stops, cue offsets |
| `calibration.py` | car model + calibration tables (defaults are estimates), `calibration.json` load/save |
| `params.py` | behaviour only: speeds, ramps, pauses, gaze, wiggle |
| `geometry.py` | segments, the one heading-sign rule, mirror/retrace, exact bicycle-model integration, footprint clearance |
| `paths.py` | MAIN / ALT / CLOSE, LH pivots and lanes, the obstacle event, the legs of every profile |
| `solver.py` | grid search that re-solves the curves for a new layout or car |
| `plan.py` | **laptop**: checks every leg (end pose, clearances, edges, radii, tape geometry), PNG plots, `--solve` |
| `motion.py` | distance→time and radius→steering from the calibration tables |
| `executor.py` | runs segment lists, ramps, steering timing, sensor stops, tape stops, command log, `retrace` |
| `profiles.py` | a whole take: legs + gaze, pauses, wiggle, operator cue; tape-strip positions |
| `sim.py` | `ArenaSim`: SimCar + layout, sliding obstacle, floor strips, servo slew, picarx's inner-wheel slowdown |
| `run.py` | **car**: run a take or one leg; `--sim`, `--dry-run`, `--speed-scale`; CSV log |
| `calibrate.py` | **car**: interactive calibration → `calibration.json` |
| `repeat.py` | **car**: repeatability (run one leg N times, enter landings, get a recommendation) |
| `tests/` | `unittest`, runs on a laptop without hardware |

## Conventions

* Units cm and degrees. Origin = bottom-left arena corner; +x from A toward the
  targets; +y up toward D. Heading 0 faces +x, positive is counter-clockwise.
* A **pose** is the car's geometric centre. The kinematics use the rear-axle
  centre (bicycle model); **turning radii are rear-axle radii**.
* A **segment** is `F|B  L|R|0  R<radius>  <amount>`: `F R R30 45` = forward,
  right steering, radius 30, 45° of heading change; `F 0 8` = straight 8 cm.
  Forward+left turns counter-clockwise, reverse+left turns clockwise
  (`geometry.heading_sign`, the only place this rule lives).
* **Mirror** (A→B ⇒ A→D) swaps L/R. **Retrace** (A→B ⇒ B→A) reverses the order
  and flips F/B, keeping steering and radius.
* picarx v2.0 (checked against the source): `set_dir_servo_angle(+)` steers
  right (±30), `set_cam_pan_angle(+)` pans right, `forward/backward(s)` send
  PWM = int(s/2)+50 and slow the **inner** rear wheel to s·(100−|angle|)/100
  when steering, `get_distance()` cm (−1/−2 errors), `get_grayscale_data()`
  three ADC values (lower = darker).

## Paths (A→B versions)

| Curve | Segments | Clearance to O0 |
|---|---|---|
| MAIN | `F 0 8 → F R R30 49 → F 0 0.59 → F L R85 49 → F 0 9.82` | 12.8 cm |
| ALT | `F 0 8 → F R R30 58 → F 0 2.8 → F L R50 58 → F 0 27.7` | 12.8 cm |
| CLOSE | `F 0 10 → F R R40 47 → F 0 6.9 → F L R70 47 → F 0 9.9` | 6.7 cm (on purpose) |

MAIN and ALT start and end with a straight at least `ramp_cm` (8 cm) long:
speed ramps never happen inside an arc, and the retraced legs end on a
straight, so the tape stop at A works. MAIN keeps the long sweeping arc. ALT
is chosen to look **different** from MAIN on camera (HL legs 1 and 2): a
steeper dive, an early flatten and a long final straight. It is selected by
the largest sideways distance from MAIN's path (`geometry.path_difference`,
5.3 cm) with at least 9 cm to O0; `plan.py` prints this difference. The
brief's versions are kept as `MAIN_BRIEF` / `ALT_BRIEF`.

## Setup

On the car (Raspberry Pi, SunFounder picarx installed), from the repo root:

```bash
python3 -m unittest discover -s trajectory_profiles/tests -t .   # includes the picarx API check
```

On the laptop (Python 3; `matplotlib` only for the PNGs):

```bash
python -m unittest discover -s trajectory_profiles/tests -t .
python trajectory_profiles/plan.py --png        # table + plots/HH.png, HL.png, LH.png
python trajectory_profiles/run.py HH --sim      # a whole take in the simulator
```

Run the car over SSH from the laptop: the operator's **PULL** cue (terminal
bell + big text) then appears on the laptop. The robot never beeps.

## Calibration order

Run `python3 trajectory_profiles/calibrate.py` on the car (or `--only <check>`).
Each check saves to `calibration.json` (git-ignored, one per car).

**Calibration never blocks a run.** `run.py` and `repeat.py` always run,
whether `calibrate.py` was never run, partly failed or fully passed:

* every value whose check **passed** is used; anything missing or failed falls
  back to its default. The startup message lists exactly which values are
  defaults and why (check not run / check failed), and the run's CSV log has
  `calibration` rows with the measured and default values;
* speed is capped at 50 % **only** while `speed_table` or `radius_table` is
  missing or failed. `--no-cap` removes the cap (with a one-line warning);
* other checks only affect their own feature: no `grayscale_threshold` →
  tape stops are off and legs end on timed stops; no `grayscale_offset`,
  `servo_swing`, `footprint` or `steering_trim` → a warning, estimates used;
* `car.ramp_cm`, `speed_tau_s`, `servo_settle_s` and `stop_settle_s` are
  estimates that no check measures (also listed at startup).

`run.py` still refuses if `plan.py`'s geometry checks fail for the car model
(e.g. a measured footprint too wide for CLOSE): that is a path-safety rule,
not a calibration gate.

1. **api**: the installed picarx has every method used here.
2. **ultrasonic**: 30 readings at 30 cm.
3. **motors**: forward then reverse blip.
4. **footprint**: measure rear bumper, front bumper and ultrasonic face from the
   rear-axle centre, and the overall width. This sets the car centre.
5. **steering_trim**: drive 150 cm straight, enter the sideways drift; repeat
   until it is under 2 cm.
6. **speed_table**: for each command, two timed straight runs; enter both
   distances. Gives cm/s and the start/stop lag per command. If the start-up
   distance is longer than `ramp_cm`, it says so.
7. **radius_table**: one right arc per steering angle (8–30°); enter where the
   rear-axle centre ended (forward x, sideways y). Gives the radius and the
   arc-speed correction. Arcs instead of full circles so the R120 angle fits in
   the arena. Sets `min_radius`; if it is above 30 cm, run `plan.py --solve`.
8. **servo_swing**: film the wheels in slow motion; enter one −30→+30 swing time.
9. **grayscale_threshold**: floor vs. tape readings → per-channel threshold.
10. **grayscale_offset**: slide a strip under the module (static offset), then
    optionally drive at a strip at ~17, 35 and 50 cm/s to measure how far the
    car coasts past it (`tape_overshoot`).

Then copy `calibration.json` to the laptop and run
`python trajectory_profiles/plan.py --calibration calibration.json`: with the
measured car, every geometry check must still pass (run.py refuses otherwise).

## Floor-taping guide

All coordinates in cm from the bottom-left corner of the 150 × 150 arena.

| Mark | Where |
|---|---|
| A (start) | (20, 75): car centre; car faces +x |
| B, C, D | (125, 35), (125, 75), (125, 115) |
| Obstacle start O0 | centre (80, 75), so its footprint is x 72.5–87.5, y 67.5–82.5 |
| Obstacle end O1 | centre (80, 45): it slides 30 cm in −y |
| Eyelet | fixed on the south edge at x = 80 (y = 0) |
| Bead / knot | on the line so the obstacle stops exactly at O1 (centre y = 45, i.e. its south face at y = 37.5) |
| **East side** | **no wall or object within 40 cm east of x = 150** (i.e. nothing at x < 190) |
| Other edges | keep walls, if any, outside 150 × 150 |

**Why the east side must be open:** arriving at B, C or D the car faces +x
with its ultrasonic about 13 cm ahead of the centre (x ≈ 138). A wall at
x = 150 would read ~12 cm, right at the hard-stop distance, so the car could
abort at the targets. Nothing within 40 cm east of the arena keeps every
reading there above ~50 cm.

**Tape strips** (dark tape, 2.5 cm wide along x, 12 cm long along y, centred on
the target's y). Print the exact positions for the current calibration with
`python3 trajectory_profiles/run.py <profile> --dry-run`. With the default
(uncalibrated) values they are:

| Strip | HH | HL | LH |
|---|---|---|---|
| A (reached in reverse) | x = 30.9 | x = 30.8 | x = 31.4 |
| B, C, D (reached forward) | x = 134.1 | x = 134.2 | x = 133.6 |

A strip sits at `target_x + grayscale_offset`, shifted by the measured coast
(`tape_overshoot`) against the direction of travel, so the car *comes to rest*
on the target. The check is armed only on the last straight of a leg (at most
its last 15 cm), so strips crossed earlier are ignored; if no strip is seen
within 4 cm past the plan, the car stops anyway and logs `TAPE_MISSED`. Each
target's tape stop can be switched off or put in `last_n` mode in
`layout.py` (`Layout.tape`).

## First-run checklist

1. `plan.py` passes on the laptop with the car's `calibration.json`.
2. Floor taped (targets, strips, eyelet, bead); obstacle at O0; line runs
   through the eyelet to the operator.
3. **Keep the area behind A clear.** Reverse moves have no sensor (B→A, C→A,
   D→A and every LH retrace end by backing into A); nothing and nobody behind
   the car.
4. Start slow: `run.py HH --leg 1` (50 % automatically while the speed or
   radius table is missing; `--speed-scale 0.5` otherwise).
5. Test **one leg at a time** (`--leg N`, car placed at that leg's start, obstacle
   where that leg needs it). Watch the PULL cue timing on the event legs.
6. Run **5 repeats per leg** with `repeat.py <profile> <leg>` and follow its
   recommendation (tape the X / enable the tape stop / tune the curve).
7. Then full takes: `run.py HH`. A take is clean only if the script ends with
   `take completed` (no `RETAKE`, `TAPE_MISSED`, `NO_OBSTACLE`, abort or
   e-stop). Every run writes `logs/<profile>_<timestamp>.csv`.

Emergency stop at any moment: **Enter**, Ctrl+C or SIGTERM. The motors stop and
every servo centres on any error, abort or interrupt.

## Operator cue (obstacle pull)

| Profile | Cue | Robot |
|---|---|---|
| HH | PULL at t = 0 | starts the A→C drive at +0.3 s, stops ≈1.0 s |
| HL | PULL at t = 0 | starts at once, stops ≈1.4 s |
| LH | PULL when the first pivot of A→C is done | must be at O1 before the car enters the lower lane (~2.5 s later) |

Offsets live in `Layout.cue`. At the HH/HL stop, the ultrasonic confirms the
obstacle: a validated reading above 27 cm logs `OBSTACLE_LATE` and **ends the
take** (`RETAKE`). The car is already stopped and does not back up or re-plan,
because the re-plan to C crosses O0, where a late obstacle still is. Reset and
run the take again.

On LH the car stops on **2** consecutive valid readings below 20 cm on the
lower lane (it normally rests ~12–13 cm into the lane). If nothing shows up
within **20 cm** of entering the lane it stops anyway (`NO_OBSTACLE`) and still
retraces to A; at that cap it is 9.5 cm from the obstacle's slide strip
(`plan.py` checks ≥ 8 cm).

## How the executor times things

* Distance → time from `speed_table` (per command, with a start/stop lag). Arc
  speed is the mean of the outer wheel and picarx's slowed inner wheel, times a
  calibrated correction. Each segment uses the command that gives the target
  ground speed at its steering angle, and is timed with the speed that command
  *really* gives (picarx has a minimum speed, ≈16 cm/s by default).
* **No speed changes inside an arc**: ramps are steps on the first and last
  straight of a run only.
* LH: stop → steer → wait swing + settle → go, for every segment. HH/HL: each
  steering change is issued half a swing before the segment boundary, together
  with the next speed command (picarx applies the inner-wheel slowdown for the
  new angle immediately). A straight too short for the servo (e.g. 0.6 cm
  between MAIN's arcs) becomes part of a single swing.
* The car waits `stop_settle_s` after a stop before steering or reversing.
* Every segment is logged with its actual time and the distance that
  corresponds to; `retrace(log)` drives that backwards with the same model.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `run.py` refuses: "plan.py checks FAILED" | the measured car breaks a clearance, edge or radius rule. Run `plan.py` to see which; `plan.py --solve` for new curves (paste them into `paths.py` by hand). |
| Lands short/long but on the line | speed table or lag off; enable the tape stop or redo `speed_table`. `repeat.py` says "enable the tape stop". |
| Lands to one side | steering trim or radius table; redo `steering_trim` / `radius_table`. `repeat.py` says "tune the curve". |
| Error grows with speed (HH, not HL) | servo swing time or `speed_tau_s` in `calibration.json`; redo `servo_swing`. |
| `TAPE_MISSED` | strip not under the module: check `run.py --dry-run` positions and `grayscale_threshold` on the real floor and light. |
| Stops early on a forward leg (`HARD_STOP` abort) | something within 12 cm of the ultrasonic. Check the east side: no wall within 40 cm of x = 150. |
| `RETAKE` (`OBSTACLE_LATE`) | the pull came late or the obstacle didn't reach the bead. The take ended at the stop; reset and run it again. |
| Startup lists many DEFAULTS | expected before calibration; run the missing checks. Speed is capped only for `speed_table` / `radius_table` (`--no-cap` to override). |
| `NO_OBSTACLE` on LH | obstacle not at O1 before the lower lane, or the reading came too late: check the cue and the ultrasonic. |
| Wobbly LH pivots | `stop_settle_s` / `servo_settle_s` too short for this floor. |

## Known limits (from the simulator)

* At HH's 50 cm/s, MAIN lands ~2.5 cm to the side with ~1° heading in the
  simulator (steering-swing shape at full lock); HL and LH land within ~1 cm.
  Tape stops absorb the along-track part; `repeat.py` shows what the real car does.
* LH: the reading drops below 20 cm ~9.5 cm into the lower lane; with 2
  confirmations and the coast the car rests ~12–13 cm in, well before the
  20 cm cap.
* The planned (never driven) LH A→C detour can only clear O0 by 5.3 cm with
  R30 pivots; it is drawn in the plot only.
* The HH/HL event starts on a full-lock arc from standstill. For HL this is
  more abrupt than its other legs (turns at ~48°/s from rest); a lead-in
  straight has been proposed but not applied.
