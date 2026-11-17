#!/usr/bin/env python3
"""Overhead-camera ArUco tracker: image pixels -> arena cm, and the car's pose.

    python tools/vision/tracker.py --selftest           # synthetic frames, no hardware
    python tools/vision/tracker.py --image photo.jpg    # one photo (later, with the real setup)

Two maps from image to arena:
  * FLOOR map, from the 4 corner markers (needs >= 3 of them visible). Exact
    for things at the corner markers' height.
  * ROOF map, for the car's roof marker, which is ~12 cm above the floor and
    therefore seen with parallax (several cm near the arena edges). Calibrate
    it once by placing the car on A, B, C and D (taped spots, known poses):
    `fit_roof_map`. Until then the floor map is used and the result is
    flagged `parallax_uncorrected`.

Pose = car CENTRE (x, y in cm, heading in degrees, 0 = +x, CCW +), the same
convention as trajectory_profiles. No live camera code yet: frames come
from images (the webcam loop is a later step and needs your OK first).
"""
import argparse
import json
import math
import os
import sys
from collections import namedtuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from markers import DICTIONARY, LAYOUT, corner_points  # noqa: E402

Pose = namedtuple("Pose", "x y h")
HERE = os.path.dirname(os.path.abspath(__file__))
ROOF_MAP_PATH = os.path.join(HERE, "roof_map.json")      # per setup: git-ignored


def car_marker_points(pose, layout=LAYOUT):
    """Arena (x, y) of the roof marker's 4 corners for a car-centre pose."""
    h = math.radians(pose.h)
    ox, oy = layout.car_offset_cm
    mx = pose.x + ox * math.cos(h) - oy * math.sin(h)
    my = pose.y + ox * math.sin(h) + oy * math.cos(h)
    fx, fy = math.cos(h), math.sin(h)
    rx, ry = math.sin(h), -math.cos(h)
    s = layout.car_size_cm / 2
    return [(mx + s * (a * rx + b * fx), my + s * (a * ry + b * fy)) for a, b in ((-1, 1), (1, 1), (1, -1), (-1, -1))]


def _apply(H, pts):
    p = cv2.perspectiveTransform(np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2), H)
    return p.reshape(-1, 2)


class ArenaTracker:
    def __init__(self, layout=LAYOUT, min_corner_markers=3):
        self.layout = layout
        self.min_corners = min_corner_markers
        params = cv2.aruco.DetectorParameters()
        # AprilTag-style corner refinement: on a ~25 px roof marker it gave the
        # best heading in the synthetic tests (worst 1.2°, mean 0.4°; sub-pixel:
        # 5.4°/2.2°; contour: 2.5°/1.0°).
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY)),
                                                params)
        self.floor_H = None          # image -> arena, corner-marker plane
        self.floor_rms_cm = None
        self.roof_H = None           # image -> arena, roof-marker plane
        self.roof_rms_cm = None

    # ---------------------------------------------------------- detection
    def detect(self, frame):
        """{marker id: 4x2 image points (TL, TR, BR, BL)}."""
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}

    # ---------------------------------------------------------- maps
    def fit_floor_map(self, det):
        """Fit the floor map from the corner markers seen in `det`. Returns the
        RMS error (cm) of the corner points, or None if too few are visible."""
        world = corner_points(self.layout)
        seen = [i for i in world if i in det]
        if len(seen) < self.min_corners:
            return None
        img = np.vstack([det[i] for i in seen])
        arena = np.vstack([world[i] for i in seen])
        H, _ = cv2.findHomography(img, arena, 0)
        self.floor_H = H
        self.floor_rms_cm = float(np.sqrt(np.mean(np.sum((_apply(H, img) - arena) ** 2, axis=1))))
        return self.floor_rms_cm

    def fit_roof_map(self, samples):
        """samples: [(detections, known car-centre Pose)], e.g. the car placed
        on A, B, C and D. Fits the roof-height map; returns its RMS error (cm)."""
        img, arena = [], []
        for det, pose in samples:
            if self.layout.car_id in det:
                img.append(det[self.layout.car_id])
                arena.append(car_marker_points(pose, self.layout))
        if len(img) < 2:
            raise ValueError("need the car marker in at least 2 placements (better: A, B, C and D)")
        img, arena = np.vstack(img), np.vstack(arena)
        H, _ = cv2.findHomography(img, arena, 0)
        self.roof_H = H
        self.roof_rms_cm = float(np.sqrt(np.mean(np.sum((_apply(H, img) - arena) ** 2, axis=1))))
        return self.roof_rms_cm

    def save_roof_map(self, path=ROOF_MAP_PATH):
        with open(path, "w") as f:
            json.dump({"H": self.roof_H.tolist(), "rms_cm": self.roof_rms_cm}, f, indent=2)

    def load_roof_map(self, path=ROOF_MAP_PATH):
        try:
            with open(path) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return False
        self.roof_H, self.roof_rms_cm = np.array(d["H"]), d.get("rms_cm")
        return True

    # ---------------------------------------------------------- pose
    def car_pose(self, det):
        """(Pose of the car centre, info dict), or (None, info) if the car
        marker or a map is missing."""
        info = {"seen": sorted(det), "parallax_uncorrected": False}
        if self.layout.car_id not in det:
            return None, dict(info, reason="car marker not visible")
        H = self.roof_H
        if H is None:
            H = self.floor_H
            info["parallax_uncorrected"] = self.layout.car_height_cm != self.layout.corner_height_cm
        if H is None:
            return None, dict(info, reason="no map: corner markers not fitted yet")
        p = _apply(H, det[self.layout.car_id])
        centre = p.mean(axis=0)
        up = (p[0] + p[1]) / 2 - (p[3] + p[2]) / 2          # top edge - bottom edge = car forward
        h = math.atan2(up[1], up[0])
        ox, oy = self.layout.car_offset_cm
        x = centre[0] - ox * math.cos(h) + oy * math.sin(h)
        y = centre[1] - ox * math.sin(h) - oy * math.cos(h)
        return Pose(float(x), float(y), math.degrees(h)), info

    def process(self, frame):
        """One frame: detect, refresh the floor map if possible, return the pose."""
        det = self.detect(frame)
        if self.fit_floor_map(det) is None and self.floor_H is None and self.roof_H is None:
            return None, {"seen": sorted(det), "reason": "fewer than %d corner markers" % self.min_corners}
        return self.car_pose(det)


# ------------------------------------------------------------------ self-test
# Single-frame accuracy the tracker must reach on synthetic frames: well inside
# the 2 cm / 2° the tuning loop aims for.
POS_TOL_CM, HEADING_TOL_DEG = 1.0, 2.0
TARGETS = {"A": Pose(20, 75, 0), "B": Pose(125, 35, 0), "C": Pose(125, 75, 0), "D": Pose(125, 115, 0)}
TEST_POSES = [Pose(20, 75, 0), Pose(125, 35, 0), Pose(75, 75, 37), Pose(40, 130, -120), Pose(140, 10, 170),
              Pose(10, 10, 90), Pose(100, 60, -45)]


def selftest(verbose=True):
    """Synthetic check of the whole chain. Returns a dict of worst errors."""
    from synthetic import render
    tr = ArenaTracker()
    rms = tr.fit_floor_map(tr.detect(render()))
    out = {"floor_rms_cm": rms}
    # 1) floor map only: shows the parallax on the 12 cm-high roof marker
    worst_floor = 0.0
    for p in TEST_POSES:
        est, _ = tr.car_pose(tr.detect(render(car=p)))
        worst_floor = max(worst_floor, math.hypot(est.x - p.x, est.y - p.y))
    # 2) roof map from the car placed on A, B, C, D
    tr.fit_roof_map([(tr.detect(render(car=t, seed=k)), t) for k, t in enumerate(TARGETS.values())])
    worst_pos = worst_h = 0.0
    rows = []
    for k, p in enumerate(TEST_POSES):
        est, info = tr.car_pose(tr.detect(render(car=p, seed=10 + k)))
        dpos = math.hypot(est.x - p.x, est.y - p.y)
        dh = abs((est.h - p.h + 180) % 360 - 180)
        worst_pos, worst_h = max(worst_pos, dpos), max(worst_h, dh)
        rows.append((p, est, dpos, dh))
    # 3) one corner marker hidden
    tr3 = ArenaTracker()
    rms3 = tr3.fit_floor_map(tr3.detect(render(hide=(2,))))
    out.update(floor_only_worst_cm=worst_floor, roof_rms_cm=tr.roof_rms_cm, worst_pos_cm=worst_pos,
               worst_heading_deg=worst_h, three_corner_rms_cm=rms3)
    if verbose:
        print("floor map from 4 corner markers: RMS %.2f cm" % rms)
        print("roof marker with the FLOOR map only (parallax): worst %.1f cm" % worst_floor)
        print("roof map from the car on A, B, C, D: RMS %.2f cm" % tr.roof_rms_cm)
        print("%-24s %-28s %8s %8s" % ("true pose", "tracked", "pos cm", "hdg deg"))
        for p, e, dpos, dh in rows:
            print("(%6.1f, %6.1f, %6.1f)   (%6.1f, %6.1f, %6.1f)     %6.2f %8.2f" % (p + e + (dpos, dh)))
        print("worst: %.2f cm, %.2f deg" % (worst_pos, worst_h))
        print("with one corner marker hidden: floor map RMS %s" % ("%.2f cm" % rms3 if rms3 is not None else "FAILED"))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="run on synthetic frames")
    ap.add_argument("--image", help="process one image file")
    args = ap.parse_args(argv)
    if args.selftest:
        r = selftest()
        ok = r["worst_pos_cm"] < POS_TOL_CM and r["worst_heading_deg"] < HEADING_TOL_DEG
        print("SELFTEST", "PASS" if ok else "FAIL")
        return 0 if ok else 1
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print("cannot read", args.image)
            return 1
        tr = ArenaTracker()
        loaded = tr.load_roof_map()
        pose, info = tr.process(frame)
        print("markers seen:", info.get("seen"), "| floor map RMS:", tr.floor_rms_cm,
              "| roof map:", "loaded" if loaded else "none (parallax uncorrected)")
        print("car pose:", pose if pose else info.get("reason"))
        return 0 if pose else 1
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
