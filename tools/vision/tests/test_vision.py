"""Vision tests on synthetic frames (laptop; skipped without OpenCV):

    python -m unittest discover -s tools/vision/tests -t tools/vision
"""
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

try:
    import cv2  # noqa: F401
    HAVE_CV2 = hasattr(cv2, "aruco")
except ImportError:
    HAVE_CV2 = False


@unittest.skipUnless(HAVE_CV2, "opencv-contrib-python not installed")
class Vision(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import tracker
        from synthetic import render
        cls.T, cls.render = tracker, staticmethod(render)

    def tracker_with_maps(self):
        tr = self.T.ArenaTracker()
        self.assertLess(tr.fit_floor_map(tr.detect(self.render())), 0.3)
        tr.fit_roof_map([(tr.detect(self.render(car=t, seed=k)), t) for k, t in enumerate(self.T.TARGETS.values())])
        return tr

    def test_markers_decode_with_the_right_ids(self):
        tr = self.T.ArenaTracker()
        self.assertEqual(sorted(tr.detect(self.render(car=self.T.Pose(75, 75, 0)))), [0, 1, 2, 3, 10])

    def test_pose_accuracy_with_roof_map(self):
        tr = self.tracker_with_maps()
        for k, p in enumerate(self.T.TEST_POSES):
            est, info = tr.car_pose(tr.detect(self.render(car=p, seed=20 + k)))
            self.assertFalse(info["parallax_uncorrected"])
            self.assertLess(math.hypot(est.x - p.x, est.y - p.y), self.T.POS_TOL_CM, p)
            self.assertLess(abs((est.h - p.h + 180) % 360 - 180), self.T.HEADING_TOL_DEG, p)

    def test_floor_map_alone_shows_parallax_and_flags_it(self):
        tr = self.T.ArenaTracker()
        tr.fit_floor_map(tr.detect(self.render()))
        p = self.T.Pose(140, 10, 170)
        est, info = tr.car_pose(tr.detect(self.render(car=p)))
        self.assertTrue(info["parallax_uncorrected"])
        self.assertGreater(math.hypot(est.x - p.x, est.y - p.y), 2.0)   # why the roof map exists

    def test_three_corner_markers_are_enough(self):
        tr = self.T.ArenaTracker()
        self.assertIsNotNone(tr.fit_floor_map(tr.detect(self.render(hide=(2,)))))
        tr2 = self.T.ArenaTracker()
        self.assertIsNone(tr2.fit_floor_map(tr2.detect(self.render(hide=(1, 2)))))

    def test_missing_car_or_map(self):
        tr = self.T.ArenaTracker()
        pose, info = tr.car_pose(tr.detect(self.render()))
        self.assertIsNone(pose)
        self.assertIn("not visible", info["reason"])
        pose, info = tr.car_pose(tr.detect(self.render(car=self.T.Pose(75, 75, 0))))
        self.assertIsNone(pose)
        self.assertIn("no map", info["reason"])

    def test_roof_map_save_and_load(self):
        tr = self.tracker_with_maps()
        path = os.path.join(tempfile.mkdtemp(), "roof_map.json")
        tr.save_roof_map(path)
        tr2 = self.T.ArenaTracker()
        self.assertTrue(tr2.load_roof_map(path))
        self.assertTrue((abs(tr2.roof_H - tr.roof_H) < 1e-9).all())

    def test_printable_sheets(self):
        import markers
        d = tempfile.mkdtemp()
        markers.write_pdf(os.path.join(d, "m.pdf"), preview_dir=d)
        tr = self.T.ArenaTracker()
        found = []
        for k in range(1, 6):
            img = cv2.imread(os.path.join(d, "page%d.png" % k), cv2.IMREAD_GRAYSCALE)
            det = tr.detect(img)
            self.assertEqual(len(det), 1)
            (mid, c), = det.items()
            self.assertLess(c[0][1], c[3][1])          # top edge is up on the page (the arrow)
            found.append(mid)
        self.assertEqual(found, [0, 1, 2, 3, 10])


if __name__ == "__main__":
    unittest.main()
