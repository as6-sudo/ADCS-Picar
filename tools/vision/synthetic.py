"""Synthetic overhead-camera frames for testing the tracker without hardware.

A pinhole camera above the arena looks down at the corner markers (on the
floor, or raised) and the roof marker (at car height), so the images have
the same perspective and parallax as the real setup. Nothing here touches a
real camera.
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from markers import BITS, DICTIONARY, LAYOUT


@dataclass(frozen=True)
class Camera:
    width: int = 1280
    height: int = 720
    focal_px: float = 650.0             # ~89° x 58° field of view, a typical webcam
    position: tuple = (75.0, 75.0, 200.0)   # cm above the arena centre (>= 180 level; see README)
    tilt_deg: tuple = (2.0, -1.5, 1.0)      # small rotations about x, y, z: not perfectly level

    def matrices(self):
        K = np.array([[self.focal_px, 0, self.width / 2], [0, self.focal_px, self.height / 2], [0, 0, 1.0]])
        # Looking straight down: camera x = arena +y (long image side across the
        # arena's y), camera y = arena +x, camera z = down.
        base = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
        rx, ry, rz = (math.radians(a) for a in self.tilt_deg)
        Rt = cv2.Rodrigues(np.array([rx, ry, rz]))[0]
        R = Rt @ base
        t = -R @ np.array(self.position, dtype=float)
        return K, R, t

    def project(self, pts3):
        K, R, t = self.matrices()
        pc = (R @ np.asarray(pts3, dtype=float).T).T + t
        uv = (K @ pc.T).T
        return uv[:, :2] / uv[:, 2:3]


def marker_corners_3d(cx, cy, size, heading_deg, z):
    """Corners (TL, TR, BR, BL of the printed marker) in arena cm, with the
    marker's top edge facing `heading_deg` (0 = +x)."""
    h = math.radians(heading_deg)
    fx, fy = math.cos(h), math.sin(h)            # "up" on the printed marker
    rx, ry = math.sin(h), -math.cos(h)           # "right" on the printed marker
    s = size / 2
    out = []
    for a, b in ((-1, 1), (1, 1), (1, -1), (-1, -1)):   # (right, up) signs
        out.append((cx + s * (a * rx + b * fx), cy + s * (a * ry + b * fy), z))
    return out


def render(car=None, camera=Camera(), layout=LAYOUT, hide=(), blur=1.0, noise=3.0, seed=0):
    """Grey image of the arena markers and (optionally) the roof marker.

    car: (x, y, heading_deg) of the car CENTRE. The roof marker sits at
    layout.car_offset_cm from it and layout.car_height_cm above the floor."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    img = np.full((camera.height, camera.width), 235, np.uint8)
    items = [(i, cx, cy, layout.corner_size_cm, 90.0, layout.corner_height_cm)
             for i, (cx, cy) in layout.corners.items() if i not in hide]
    if car is not None:
        x, y, hd = car
        h = math.radians(hd)
        ox, oy = layout.car_offset_cm
        mx = x + ox * math.cos(h) - oy * math.sin(h)
        my = y + ox * math.sin(h) + oy * math.cos(h)
        items.append((layout.car_id, mx, my, layout.car_size_cm, hd, layout.car_height_cm))
    cells = BITS + 2
    for mid, cx, cy, size, heading, z in items:
        q = size / cells
        # white quiet zone (one cell), then the marker
        for sz, tile in ((size + 2 * q, np.full((60, 60), 255, np.uint8)),
                         (size, cv2.aruco.generateImageMarker(dictionary, mid, cells * 40))):
            dst = camera.project(marker_corners_3d(cx, cy, sz, heading, z)).astype(np.float32)
            n = tile.shape[0] - 1
            src = np.float32([[0, 0], [n, 0], [n, n], [0, n]])
            H = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(tile, H, (camera.width, camera.height), flags=cv2.INTER_LINEAR)
            mask = cv2.warpPerspective(np.full_like(tile, 255), H, (camera.width, camera.height))
            img[mask > 127] = warped[mask > 127]
    if blur:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    if noise:
        rng = np.random.default_rng(seed)
        img = np.clip(img.astype(float) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img
