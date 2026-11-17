"""ArUco marker layout for the overhead-camera tracker, and the printable sheets.

    python tools/vision/markers.py              # writes tools/vision/aruco_markers.pdf

Arena frame (same as trajectory_profiles): cm, origin = bottom-left corner of
the 150 x 150 arena, +x from A toward the targets, +y toward D, heading 0 =
facing +x, counter-clockwise positive.

* 4 CORNER markers just outside the arena corners give the pixel -> cm map.
  Each one is placed with the printed arrow (the marker's top edge) pointing
  to +y, i.e. toward D's side of the arena.
* 1 CAR marker on the roof gives the car's position and heading. It is placed
  with the printed arrow pointing to the car's FRONT.

Every number below is a design default; measure the real placement and
update it here.
"""
import os
from dataclasses import dataclass, field

DICTIONARY = "DICT_4X4_50"      # few, large cells: robust at a distance and with motion blur
BITS = 4                        # data cells per side; the printed marker is BITS + 2 cells wide


@dataclass(frozen=True)
class MarkerLayout:
    corner_size_cm: float = 10.0        # printed black square, side length
    # Corner marker centres, 9 cm outside each arena corner along the diagonal:
    # far enough out that the car (and its tape strips) never cover them.
    corners: dict = field(default_factory=lambda: {
        0: (-9.0, -9.0),        # SW, next to (0, 0)
        1: (159.0, -9.0),       # SE, next to (150, 0)
        2: (159.0, 159.0),      # NE, next to (150, 150)
        3: (-9.0, 159.0),       # NW, next to (0, 150)
    })
    car_id: int = 10
    car_size_cm: float = 7.0            # fits the PiCar-X roof with a white margin
    # Where the car marker's centre is relative to the car's geometric centre,
    # cm: (+ forward, + to the car's LEFT). Measure after mounting it.
    car_offset_cm: tuple = (0.0, 0.0)
    # Heights above the floor. The camera sees the roof marker with parallax;
    # the map is exact for markers at `corner_height_cm`. Best: raise the corner
    # markers to the roof marker's height (e.g. on boxes) so both are equal.
    corner_height_cm: float = 0.0       # corner markers flat on the floor (default)
    car_height_cm: float = 12.0         # roof marker height (estimate; measure)


LAYOUT = MarkerLayout()


def corner_points(layout=LAYOUT):
    """{id: 4x2 arena points} of each corner marker's corners in OpenCV's
    order (top-left, top-right, bottom-right, bottom-left of the printed
    marker), with the marker's top edge facing +y."""
    s = layout.corner_size_cm / 2
    return {i: [(cx - s, cy + s), (cx + s, cy + s), (cx + s, cy - s), (cx - s, cy - s)]
            for i, (cx, cy) in layout.corners.items()}


# ------------------------------------------------------------------ printing
def write_pdf(path, layout=LAYOUT, preview_dir=None):
    """Printable sheets: one marker per page (4 corners, then the car). Every page has a 10 cm scale bar: print at 100 % / 'actual size' and
    check it with a ruler before using the markers. With `preview_dir`, each
    page is also saved as a PNG for checking on screen."""
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    page_w, page_h = 21.0, 27.9          # cm: fits on both A4 (21 x 29.7) and US Letter (21.6 x 27.9)
    names = {0: "SW corner", 1: "SE corner", 2: "NE corner", 3: "NW corner"}

    def draw(ax, mid, cx, cy, size, title, note):
        cells = BITS + 2
        img = cv2.aruco.generateImageMarker(dictionary, mid, cells * 20)
        q = size / cells                             # quiet zone: one cell of white around it
        ax.add_patch(plt.Rectangle((cx - size / 2 - q, cy - size / 2 - q), size + 2 * q, size + 2 * q,
                                   fill=False, lw=0.3, ls=":", ec="0.6"))
        ax.imshow(img, cmap="gray", vmin=0, vmax=255, interpolation="nearest",
                  extent=(cx - size / 2, cx + size / 2, cy - size / 2, cy + size / 2))
        ax.annotate("", xy=(cx + size / 2 + 1.2, cy + size / 2), xytext=(cx + size / 2 + 1.2, cy - size / 2 + 1),
                    arrowprops=dict(arrowstyle="-|>", lw=1.2, color="k"))
        ax.text(cx, cy + size / 2 + q + 0.6, title, ha="center", va="bottom", fontsize=11, weight="bold")
        ax.text(cx, cy - size / 2 - q - 0.5, note, ha="center", va="top", fontsize=8)

    pages = [0]

    def page(pdf, items):
        fig = plt.figure(figsize=(page_w / 2.54, page_h / 2.54))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(0, page_w)
        ax.set_ylim(0, page_h)
        ax.set_aspect("equal")
        ax.axis("off")
        for it in items:
            draw(ax, *it)
        # scale bar: must measure exactly 10.0 cm on paper
        ax.plot([5.5, 15.5], [1.5, 1.5], color="k", lw=2)
        for x in (5.5, 15.5):
            ax.plot([x, x], [1.2, 1.8], color="k", lw=1)
        ax.text(10.5, 1.9, "10.0 cm: print at 100 % (actual size) and check", ha="center", fontsize=7)
        ax.text(10.5, 0.8, "ArUco %s. Arrow = marker top." % DICTIONARY, ha="center", fontsize=6, color="0.4")
        pdf.savefig(fig)
        pages[0] += 1
        if preview_dir:
            os.makedirs(preview_dir, exist_ok=True)
            fig.savefig(os.path.join(preview_dir, "page%d.png" % pages[0]), dpi=60)
        plt.close(fig)

    cs = layout.corner_size_cm
    with PdfPages(path) as pdf:
        for mid in sorted(layout.corners):
            cx, cy = layout.corners[mid]
            page(pdf, [(mid, page_w / 2, 16.0, cs, "ID %d: %s" % (mid, names[mid]),
                        "centre at arena (%g, %g) cm; arrow points to +y (toward D)" % (cx, cy))])
        page(pdf, [(layout.car_id, page_w / 2, 16.0, layout.car_size_cm, "ID %d: car roof" % layout.car_id,
                    "flat on the roof; arrow points to the car's FRONT; measure its offset from the car centre")])
    return path


if __name__ == "__main__":
    out = write_pdf(os.path.join(os.path.dirname(os.path.abspath(__file__)), "aruco_markers.pdf"))
    print("wrote", out)
