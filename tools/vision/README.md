# tools/vision: overhead-camera tracking (laptop only)

Tracks the PiCar-X from an overhead webcam with ArUco markers, in the same
arena frame as `trajectory_profiles` (cm, origin = arena bottom-left, +x toward
the targets, heading 0 = +x, CCW +). It is the "eyes" for the agentic tuning
loop: run a leg, track it, compare with the plan, adjust the calibration.

Status: markers and tracker core are done and tested on **synthetic** frames.
Nothing has been mounted or filmed yet; the live webcam loop comes next.

## Setup (laptop)

```bash
python -m pip install -r tools/vision/requirements.txt
python tools/vision/tracker.py --selftest
python -m unittest discover -s tools/vision/tests -t tools/vision
```

Installed and tested with Python 3.14, opencv-contrib-python 5.1.0, numpy 2.5.
The Pi does not need OpenCV.

## Markers

`aruco_markers.pdf` (regenerate: `python tools/vision/markers.py`): dictionary
`DICT_4X4_50`, one marker per page. Print at **100 % / actual size** and check
the 10 cm scale bar on each page with a ruler.

| ID | Size | Where | Arrow (marker top) points to |
|---|---|---|---|
| 0 | 10 cm | centre (−9, −9): SW, outside arena corner (0, 0) | +y (toward D) |
| 1 | 10 cm | centre (159, −9): SE, outside (150, 0) | +y |
| 2 | 10 cm | centre (159, 159): NE, outside (150, 150) | +y |
| 3 | 10 cm | centre (−9, 159): NW, outside (0, 150) | +y |
| 10 | 7 cm | flat on the car roof | the car's front |

Keep the white border around each marker. Positions, sizes and heights live
in `markers.py` (`MarkerLayout`); update them to what you actually measure,
including the roof marker's offset from the car centre.

## Things the synthetic tests showed (for the physical setup later)

* **Parallax.** The roof marker is ~12 cm above the floor. Mapped with the
  floor markers alone, the car's position is off by up to **6 cm** near the
  edges. Fix: a second "roof map", calibrated once by placing the car on the
  taped A, B, C and D (`ArenaTracker.fit_roof_map`). With it: worst
  0.8 cm / 1.2° in a single frame, typically 0.3 cm / 0.7°. Accuracy is best
  inside the A–D area and drops slightly toward the far corners.
* **Field of view.** For a 1280 × 720 webcam with a ~89° × 58° view (typical;
  check yours), the arena plus corner markers fit across the short image side
  only with the camera **level and at least ~180 cm up**; a 4° tilt needs
  ~210 cm. Align the image's long side with the arena's y (A→D) direction or
  its x — whichever the mount allows — and leave margin.
* **Marker size in the image.** At 200 cm the 7 cm roof marker is only
  ~25 px wide (~4 px per cell). It is still detected reliably, and the
  AprilTag corner refinement keeps the heading within ~1°. A bigger roof
  marker (if the roof allows) or a 1080p camera would add margin.

## Files

| File | Purpose |
|---|---|
| `markers.py` | marker IDs, sizes, placements; writes the printable PDF |
| `tracker.py` | `ArenaTracker`: detect markers, floor map, roof map, car pose; `--selftest`, `--image` |
| `synthetic.py` | pinhole-camera renderer for tests (perspective + parallax) |
| `tests/test_vision.py` | unit tests on synthetic frames |
