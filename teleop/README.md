# teleop: drive the PiCar from the laptop keyboard, in any profile's style

The operator chooses **where** the car goes; the profile decides **how** it moves
(speed, ramps, steering smoothness, head, quirks), so each storyboard can be
acted out by hand.

```
laptop: client.py (Tk window, key-down + key-up)  --SSH stdin/stdout JSON-->  Pi: server.py -> core.py -> picarx
```

| File | Purpose |
|---|---|
| `styles.py` | per-profile driving style (HH, HL, LH, LL) |
| `core.py` | keyboard state -> motor/servo commands, safety, macros (hardware-free) |
| `server.py` | **car**: runs the core at 50 Hz, started by the client over SSH |
| `client.py` | **laptop**: the window; `--sim` runs the same core on a simulated arena |
| `tests/` | core against the simulator: `python -m unittest discover -s teleop/tests -t .` |

## Run

Laptop and Pi on the iPhone hotspot, then on the laptop from the repo root:

```
python teleop/client.py            # the real car
python teleop/client.py --sim      # practise on screen first (O pulls the obstacle, P resets)
```

The client starts the server on the Pi itself (with `ROBOT_HAT_GPIOCHIP=0`);
nothing needs to run on the Pi first. The Pi needs `teleop/` copied to
`~/ADCS-Picar/teleop/`.

## Keys (the window must have focus)

| Key | Action |
|---|---|
| **W / S** (hold) | forward / reverse. Release = stop (LL: ~1 s late) |
| **A / D** (hold) | steer. **LH: tap = 90° 3-step pivot** (stopped) |
| **Space** | STOP now, every profile, cancels any macro |
| **Esc** | stop, centre, quit |
| arrows (hold) | head pan / tilt |
| C | centre the head (and steering when stopped) |
| 1 2 3 4 | profile HH / HL / LH / LL (only when stopped) |
| [ ] | slower / faster, ±10 % of the profile's speed |
| Q / E | glance left / right (profile's head speed and hold) |
| H | happy wiggle (stopped) |
| B | surprise: the profile's reaction to the obstacle |
| M | mark the start of a leg (clears the recording) |
| R | retrace: drive the recorded commands since M backwards, back to the mark |

## Profiles

Speeds are limited by the car: the slowest it can drive is ~22 cm/s
(command 1, picarx never sends less than 50 % PWM), the fastest ~41 cm/s.

| | Speed | Driving | B (surprise) | Storyboard |
|---|---|---|---|---|
| **HH** | 41 cm/s, short ramp | quick, smooth steering to full lock | stop, back-up arc, quick look right | look (Q/E) → fast arc → land on X; R for B→A; H at D |
| **HL** | 25 cm/s, long ramp | slow steering, max 22° (gentle arcs) | pause, slow back-up, slow look | slow arcs; B then drive the new arc to C; H at C |
| **LH** | 35 cm/s, instant | straight only; A/D tap = pivot; 2 s pause after each stop | halt, 2 s, retrace to M | lanes: W, D-pivot, W, A-pivot, W; R reverses the same lane |
| **LL** | ~28 cm/s ± jitter | steering wobble, head drifts, stops ~1 s late | keeps rolling 1 s, stops, wobbly retrace | wander; R comes back on a different wobble, near A not on it |

Typical leg: put the car on A, press **M**, drive to the target, then **R** to
come back the same way (the "retraced" legs), or drive back by hand for a
"different curve back".

## Safety

* Deadman: the car moves only while W/S is held. The client sends the key
  state 20×/s; if the Pi hears nothing for 0.5 s (Wi-Fi drop, window lost
  focus, laptop asleep) it stops and cancels any macro.
* Clicking away from the window releases every key and stops the car.
* Forward motion stops on 2 readings < 12 cm and stays blocked until W is
  released. The ultrasonic only sees straight ahead, so it will **not**
  catch an obstacle the car turns into from the side, and **reverse is
  unguarded** (no rear sensor).
* If SSH drops, the server sees stdin close, stops and centres every servo.
* Log: `teleop/logs/teleop_<time>.csv` on the Pi, drive commands and events only (~1 KB/min).
