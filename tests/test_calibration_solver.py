"""Tests for image calibration homography / corner solver."""

import math
import os
import unittest

import numpy as np

from freecad.frametools import image_calibration_objects
from freecad.frametools import image_tools

from tests import helpers as h


class TestHomographyBasics(unittest.TestCase):
    def test_identity_uv_maps_to_corners(self):
        corners = h.corners_rectangle(200.0, 100.0)
        H = h.homography_from_corners(corners)
        c0, cx, c1, cy = corners
        for uv, expected in (
            ((0.0, 0.0), c0),
            ((1.0, 0.0), cx),
            ((1.0, 1.0), c1),
            ((0.0, 1.0), cy),
        ):
            p = image_tools._apply_homography_uv(uv[0], uv[1], H)
            self.assertAlmostEqual(p.x, expected.x, places=3)
            self.assertAlmostEqual(p.y, expected.y, places=3)

    def test_line_length_on_rectangle(self):
        corners = h.corners_rectangle(200.0, 100.0)
        H = h.homography_from_corners(corners)
        line = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(h.length_mm(H, line), 200.0, places=3)
        line_v = h.line_uv(1, 0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(h.length_mm(H, line_v), 100.0, places=3)


class TestTrapezoidToRectangle(unittest.TestCase):
    """Distorted quad + two length constraints → near target lengths."""

    def setUp(self):
        self.ref = h.corners_rectangle(200.0, 100.0)
        self.bottom = h.line_uv(0, 0.1, 0.0, 0.9, 0.0)
        self.left = h.line_uv(1, 0.0, 0.1, 0.0, 0.9)
        self.specs = h.targets_from_reference(
            self.ref, self.bottom, self.left)

    def test_trapezoid_two_lengths(self):
        start = h.corners_trapezoid(220.0, 160.0, 110.0, shear_top=30.0)
        H, report = h.solve_corners(start, self.specs)
        self.assertIsNotNone(H)
        for spec in self.specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM, msg=spec["label"])
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)

    def test_parallelogram_two_lengths(self):
        start = h.corners_parallelogram(210.0, 95.0, shear_x=45.0)
        H, report = h.solve_corners(start, self.specs)
        self.assertIsNotNone(H)
        for spec in self.specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)


class TestSingleLengthScale(unittest.TestCase):
    """One length constraint → uniform scaling about the quad centroid."""

    def test_uniform_scale_from_bottom_edge(self):
        corners = h.corners_rectangle(100.0, 100.0)
        line = h.line_uv(0, 0.2, 0.5, 0.8, 0.5)
        H0 = h.homography_from_corners(corners)
        current = h.length_mm(H0, line)
        target = current * 1.5
        specs = [h.length_spec(line, target)]
        H, report = h.solve_corners(corners, specs)
        self.assertIsNotNone(H)
        self.assertAlmostEqual(h.length_mm(H, line), target, delta=h.LENGTH_TOL_MM)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)
        self.assertLess(report["distortion_energy"], h.DISTORTION_TOL)

    def test_one_length_uniformly_scales_square(self):
        """Primary use case: one known length scales the whole image equally."""
        corners = h.corners_rectangle(100.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        right = h.line_uv(1, 1.0, 0.0, 1.0, 1.0)
        scale = 1.5
        target = 100.0 * scale
        specs = [h.length_spec(bottom, target)]
        H, report = h.solve_corners(
            corners, specs, line_meta=h.line_by_geo(bottom, right))
        self.assertAlmostEqual(h.length_mm(H, bottom), target, delta=h.LENGTH_TOL_MM)
        self.assertAlmostEqual(
            h.length_mm(H, right), 100.0 * scale, delta=h.LENGTH_TOL_MM)
        self.assertLess(report["distortion_energy"], h.DISTORTION_TOL)
        self.assertEqual(report["mode"], "uniform_scale")
        self.assertEqual(report["determinacy"], "unterbestimmt")
        self.assertAlmostEqual(report["scale_factor"], scale, places=4)
        c0, cx, c1, cy = h.corners_from_homography(H)
        self.assertAlmostEqual(c0.distanceToPoint(cx), target, delta=h.LENGTH_TOL_MM)
        self.assertAlmostEqual(c0.distanceToPoint(cy), target, delta=h.LENGTH_TOL_MM)

    def test_one_length_uniformly_scales_trapezoid(self):
        """Uniform scale multiplies every edge length by the same factor."""
        start = h.corners_trapezoid(200.0, 160.0, 100.0, shear_top=25.0)
        constrained = h.line_uv(0, 0.1, 0.0, 0.9, 0.0)
        reference = h.line_uv(1, 0.0, 0.1, 0.0, 0.9)
        H0 = h.homography_from_corners(start)
        scale = 1.25
        target = h.length_mm(H0, constrained) * scale
        specs = [h.length_spec(constrained, target)]
        H, report = h.solve_corners(
            start, specs,
            line_meta=h.line_by_geo(constrained, reference))
        self.assertAlmostEqual(
            h.length_mm(H, constrained), target, delta=h.LENGTH_TOL_MM)
        self.assertAlmostEqual(
            h.length_mm(H, reference),
            h.length_mm(H0, reference) * scale,
            delta=h.LENGTH_TOL_MM)
        self.assertLess(report["distortion_energy"], h.DISTORTION_TOL)


class TestCentroidScalePivot(unittest.TestCase):
    """uniform_scale / uv_scale pivot at quad centroid, not c0."""

    @staticmethod
    def _centroid_xy(corners):
        return np.mean([[c.x, c.y] for c in corners], axis=0)

    def test_uniform_scale_preserves_centroid(self):
        from freecad.frametools import image_constraint_solver as cs
        from freecad.frametools import image_homography as hg

        corners = h.corners_rectangle(100.0, 80.0, origin=(50.0, 30.0, 0.0))
        params0 = image_tools._pack_corners_xy(corners)
        z_vals = image_tools._corner_z_values(corners)
        g0 = self._centroid_xy(corners)

        params = cs._uniform_scale_params(params0, 1.5)
        g1 = self._centroid_xy(hg._corners_from_xy_params(params, z_vals))
        np.testing.assert_allclose(g1, g0, atol=1e-9)

    def test_uv_scale_preserves_centroid(self):
        from freecad.frametools import image_constraint_solver as cs
        from freecad.frametools import image_homography as hg

        corners = h.corners_trapezoid(220.0, 170.0, 110.0, shear_top=30.0)
        params0 = image_tools._pack_corners_xy(corners)
        z_vals = image_tools._corner_z_values(corners)
        g0 = self._centroid_xy(corners)

        params = cs._uv_scale_params(params0, 1.264, 1.138)
        g1 = self._centroid_xy(hg._corners_from_xy_params(params, z_vals))
        np.testing.assert_allclose(g1, g0, atol=1e-9)


class TestRealisticCalibration(unittest.TestCase):
    """Sketch-like lines on a distorted quad (not U/V edge segments)."""

    def setUp(self):
        self.start = h.corners_trapezoid(220.0, 170.0, 110.0, shear_top=30.0)
        self.H0 = h.homography_from_corners(self.start)
        # Interior lines in UV — roughly 46° apart in world XY, not u=0 / v=0.
        self.line_a = h.line_uv(0, 0.05, 0.15, 0.95, 0.35)
        self.line_b = h.line_uv(1, 0.15, 0.05, 0.45, 0.95)
        self.line_meta = h.line_by_geo(self.line_a, self.line_b)
        angle = h.angle_between_lines_deg(
            self.H0, self.line_a, self.line_b)
        self.assertGreater(angle, 35.0)
        self.assertLess(angle, 60.0)

    def test_one_length_on_interior_diagonal(self):
        """One known length on an oblique interior segment (not a U/V edge)."""
        line = h.line_uv(0, 0.15, 0.05, 0.85, 0.92)
        scale = 1.35
        target = h.length_mm(self.H0, line) * scale
        specs = [h.length_spec(line, target)]
        H, report = h.solve_corners(
            self.start, specs, line_meta=h.line_by_geo(line))
        self.assertEqual(report["mode"], "uniform_scale")
        self.assertTrue(report["success"])
        self.assertAlmostEqual(
            h.length_mm(H, line), target, delta=h.LENGTH_TOL_MM)
        self.assertLess(report["angle_preserving_energy"], h.DISTORTION_TOL)
        self.assertAlmostEqual(report["scale_factor"], scale, places=3)

    def test_two_oblique_lengths_consistent_rectangle_targets(self):
        """Two non-orthogonal lines with targets from an axis-aligned reference."""
        ref = h.corners_rectangle(200.0, 100.0)
        specs = h.targets_from_reference(ref, self.line_a, self.line_b)
        H, report = h.solve_corners(
            self.start, specs, line_meta=self.line_meta)
        for spec in specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM, msg=spec["label"])
        self.assertTrue(report["success"])
        self.assertLess(report["constraint_rank"], 6)
        self.assertTrue(report["include_distortion_energy"])
        self.assertLess(report["angle_preserving_energy"], h.DISTORTION_TOL)

    def test_two_oblique_lengths_incompatible_targets_shears(self):
        """Incompatible sx/sy targets may still need shear in phase 2."""
        specs = [
            h.length_spec(
                self.line_a, h.length_mm(self.H0, self.line_a) * 1.5),
            h.length_spec(
                self.line_b, h.length_mm(self.H0, self.line_b) * 0.7),
        ]
        H, report = h.solve_corners(
            self.start, specs, line_meta=self.line_meta)
        for spec in specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM, msg=spec["label"])
        self.assertIn(report["mode"], ("uv_scale", "corners"))
        if report["mode"] == "corners":
            self.assertTrue(report.get("uv_scale_warm_start"))
        # Phase 2 keeps angles when it exits; phase 3 may distort.
        if report["angle_preserving_energy"] > 1e-3:
            a0 = report["corner_angles_deg0"][0]
            a1 = report["corner_angles_deg"][0]
            self.assertGreater(abs(a1 - a0), 1.0)


class TestAlignImageTest1Fixture(unittest.TestCase):
    """Regression from example/align_image_test_1.FCStd (tests/fixtures/)."""

    @classmethod
    def setUpClass(cls):
        cls.fixture = h.load_fixture("align_image_test_1.json")
        cls.corners0 = h.corners_from_fixture(cls.fixture)
        cls.specs, cls.line_by_geo = h.length_specs_from_fixture(cls.fixture)
        cls.constraints = cls.fixture["constraints"]
        cls.expected = cls.fixture["solver_at_extract"]
        cls.line0 = cls.line_by_geo[0]
        cls.line1 = cls.line_by_geo[1]

    def test_fixture_lines_are_interior_sketch_segments(self):
        """L0/L1 are not the U/V boundary edges of the unit square."""
        self.assertAlmostEqual(self.line0["u0"], self.line0["u1"], places=3)
        self.assertGreater(min(self.line0["v0"], self.line0["v1"]), 0.05)
        self.assertAlmostEqual(self.line1["v0"], self.line1["v1"], places=3)
        self.assertGreater(min(self.line1["u0"], self.line1["u1"]), 0.05)

    def test_two_lengths_match_targets(self):
        H, report = h.solve_corners(
            self.corners0, self.specs,
            constraints=self.constraints,
            line_meta=self.line_by_geo)
        for spec in self.specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM, msg=spec["label"])
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)
        self.assertTrue(report["exact"])

    def test_one_length_L0_uniform_scale(self):
        """Single length 400 mm on L0 → analytic uniform scale (photo fixture)."""
        spec, constraints, line_by = h.single_length_fixture(self.fixture, 0)
        H0 = h.homography_from_corners(self.corners0)
        current = h.length_mm(H0, spec)
        expected_scale = spec["target"] / current

        H, report = h.solve_corners(
            self.corners0, [spec],
            constraints=constraints,
            line_meta=line_by)
        self.assertEqual(report["mode"], "uniform_scale")
        self.assertTrue(report["success"])
        self.assertAlmostEqual(
            h.length_mm(H, spec), spec["target"], delta=h.LENGTH_TOL_MM)
        self.assertAlmostEqual(
            report["scale_factor"], expected_scale, places=4)
        self.assertLess(report["angle_preserving_energy"], h.DISTORTION_TOL)
        self.assertFalse(report.get("uv_scale_warm_start"))

    def test_one_length_compute_calibration_from_specs(self):
        """One length via AlignedImage API on align_image_test_1 geometry."""
        spec, constraints, line_by = h.single_length_fixture(self.fixture, 0)
        doc = h.new_document("AlignImageTest1OneLen")
        try:
            img = h.make_aligned_image(doc, self.corners0)
            corners_new, H, opt_info, meta = (
                image_tools.compute_calibration_from_specs(
                    [spec], img,
                    constraints=constraints,
                    line_by_geo=line_by))
            self.assertEqual(meta.get("mode"), "uniform_scale")
            self.assertTrue(opt_info.get("success"))
            self.assertAlmostEqual(
                h.length_mm(H, spec), spec["target"], delta=h.LENGTH_TOL_MM)
            self.assertLess(
                meta.get("angle_preserving_energy", 1.0), h.DISTORTION_TOL)
            self.assertIsNotNone(corners_new)
        finally:
            h.close_document(doc)

    def test_uv_scale_warm_start_preserves_angles(self):
        """Two lengths on photo fixture: cascade stops at uv_scale when exact."""
        H, report = h.solve_corners(
            self.corners0, self.specs,
            constraints=self.constraints,
            line_meta=self.line_by_geo)
        for spec in self.specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM, msg=spec["label"])
        self.assertEqual(report["mode"], "uv_scale")
        self.assertEqual(report.get("stop_phase"), "uv_scale")
        self.assertIsNotNone(report.get("scale_sx"))
        self.assertIsNotNone(report.get("scale_sy"))
        self.assertIsNotNone(report.get("uniform_scale_factor"))
        self.assertTrue(report["success"])
        self.assertLess(report["angle_preserving_energy"], h.DISTORTION_TOL)
        self.assertLess(report["constraint_rank"], 6)
        self.assertTrue(report["include_distortion_energy"])

    def test_underdetermined_corner_solve_regression(self):
        """Former shear bug: UV scale or corners keeps E_angle ≈ 0."""
        H, report = h.solve_corners(
            self.corners0, self.specs,
            constraints=self.constraints,
            line_meta=self.line_by_geo)
        self.assertIn(report["mode"], ("uv_scale", "corners"))
        self.assertTrue(report["success"])
        self.assertLess(report["angle_preserving_energy"], h.DISTORTION_TOL)

    def test_compute_calibration_from_specs(self):
        """Same scenario through AlignedImage API."""
        doc = h.new_document("AlignImageTest1")
        try:
            img = h.make_aligned_image(doc, self.corners0)
            corners_new, H, opt_info, meta = (
                image_tools.compute_calibration_from_specs(
                    self.specs, img,
                    constraints=self.constraints,
                    line_by_geo=self.line_by_geo))
            for spec in self.specs:
                err = abs(h.length_mm(H, spec) - spec["target"])
                self.assertLess(err, h.LENGTH_TOL_MM)
            self.assertTrue(opt_info.get("success"))
            self.assertLess(
                meta.get("angle_preserving_energy", 1.0), h.DISTORTION_TOL)
            self.assertIn(meta.get("mode"), ("uv_scale", "corners"))
            if meta.get("mode") == "corners":
                self.assertTrue(meta.get("uv_scale_warm_start"))
            self.assertIsNotNone(corners_new)
        finally:
            h.close_document(doc)

    def test_vertex_labels_follow_sketch_wire_order(self):
        """V0…Vn match sketch Shape vertex order (not u/v sort)."""
        doc = h.new_document("AlignImageTest1Points")
        try:
            img = h.make_aligned_image(doc, self.corners0)
            lines_meta = h.lines_meta_with_world_from_fixture(
                self.fixture, img)
            sketch = h.sketch_from_fixture_lines(doc, img, lines_meta)
            points = image_tools._points_meta_from_sketch(sketch, img)
            self.assertEqual(len(points), 4)
            self.assertEqual(points[0]["label"], "V0")

            expected_world = []
            for local in image_tools._iter_sketch_vertex_points_local(sketch):
                expected_world.append(
                    image_tools._sketch_world_point(sketch, local))
            self.assertEqual(len(expected_world), 4)

            for pt, exp_w in zip(points, expected_world):
                self.assertLess(abs(pt["w"].x - exp_w.x), 1e-3)
                self.assertLess(abs(pt["w"].y - exp_w.y), 1e-3)
        finally:
            h.close_document(doc)

    def test_fixpoint_V0_anchors_first_sketch_vertex(self):
        """Fixed V0 must constrain the first wire vertex, not a swapped corner."""
        doc = h.new_document("AlignImageTest1FixV0")
        try:
            img = h.make_aligned_image(doc, self.corners0)
            lines_meta = h.lines_meta_with_world_from_fixture(
                self.fixture, img)
            sketch = h.sketch_from_fixture_lines(doc, img, lines_meta)
            points = image_tools._points_meta_from_sketch(sketch, img)
            point_by_index = {pt["point"]: pt for pt in points}
            v0 = point_by_index[0]
            v1 = point_by_index[1]

            H0 = h.homography_from_corners(self.corners0)
            pos0 = image_tools._apply_homography_uv(v0["u"], v0["v"], H0)
            pos1 = image_tools._apply_homography_uv(v1["u"], v1["v"], H0)
            target_x = float(pos0.x) + 10.0
            target_y = float(pos0.y)

            constraints = h.empty_constraints()
            constraints["fixed_points"] = [{
                "point": 0,
                "target_x_mm": target_x,
                "target_y_mm": target_y,
            }]

            H, report = h.solve_corners(
                self.corners0, self.specs,
                constraints=constraints,
                line_meta=self.line_by_geo,
                point_meta=point_by_index)

            after_v0 = image_tools._apply_homography_uv(
                v0["u"], v0["v"], H)
            after_v1 = image_tools._apply_homography_uv(
                v1["u"], v1["v"], H)

            self.assertLess(abs(after_v0.x - target_x), h.LENGTH_TOL_MM)
            self.assertLess(abs(after_v0.y - target_y), h.LENGTH_TOL_MM)
            self.assertGreater(
                abs(after_v1.x - target_x) + abs(after_v1.y - target_y),
                abs(after_v1.x - pos1.x) + abs(after_v1.y - pos1.y) - 1.0)
            self.assertFalse(report["include_translation_side"])
        finally:
            h.close_document(doc)

    @unittest.skipUnless(
        os.path.isfile(os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "example", "align_image_test_1.FCStd")),
        "example/align_image_test_1.FCStd missing")
    def test_vertex_labels_match_align_image_test_1_fcstd(self):
        """Regression: V labels on real example file match Shape.Vertexes."""
        import FreeCAD as App

        fcstd = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            "example", "align_image_test_1.FCStd")
        doc = App.openDocument(fcstd)
        try:
            cal = doc.getObject("ImageCalibration001")
            if cal is None:
                cal = doc.getObject("ImageCalibration")
            self.assertIsNotNone(cal)
            sketch = cal.Sketch
            source = cal.Image
            aligned = image_tools._calibration_image_for_points(cal)
            points = image_tools._points_meta_from_sketch(sketch, aligned)

            expected = [
                image_tools._sketch_world_point(sketch, v.Point)
                for v in sketch.Shape.Vertexes
            ]
            self.assertEqual(len(points), len(expected))
            for i, (pt, exp_w) in enumerate(zip(points, expected)):
                self.assertEqual(pt["label"], "V{}".format(i))
                self.assertLess(abs(pt["w"].x - exp_w.x), 1e-2)
                self.assertLess(abs(pt["w"].y - exp_w.y), 1e-2)
        finally:
            App.closeDocument(doc.Name)


class TestTwoLengthScale(unittest.TestCase):
    """Two length constraints on U/V edges → 2D scale without shear."""

    def test_two_lengths_set_width_and_height(self):
        corners = h.corners_rectangle(100.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        left = h.line_uv(1, 0.0, 0.0, 0.0, 1.0)
        specs = [
            h.length_spec(bottom, 150.0),
            h.length_spec(left, 80.0),
        ]
        H, report = h.solve_corners(
            corners, specs, line_meta=h.line_by_geo(bottom, left))
        self.assertAlmostEqual(h.length_mm(H, bottom), 150.0, delta=h.LENGTH_TOL_MM)
        self.assertAlmostEqual(h.length_mm(H, left), 80.0, delta=h.LENGTH_TOL_MM)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)
        self.assertLess(report["distortion_energy"], h.DISTORTION_TOL)

    def test_two_lengths_from_skewed_start(self):
        """Trapezoid + bottom/left targets → both lengths met."""
        start = h.corners_trapezoid(220.0, 170.0, 110.0, shear_top=30.0)
        bottom = h.line_uv(0, 0.1, 0.0, 0.9, 0.0)
        left = h.line_uv(1, 0.0, 0.1, 0.0, 0.9)
        specs = h.targets_from_reference(
            h.corners_rectangle(200.0, 100.0), bottom, left)
        H, report = h.solve_corners(
            start, specs, line_meta=h.line_by_geo(bottom, left))
        for spec in specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM, msg=spec["label"])
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)

    def test_two_lengths_rebuild_axis_aligned_rectangle(self):
        """Two targets on a square should yield a 150×80 mm rectangle."""
        corners = h.corners_rectangle(100.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        left = h.line_uv(1, 0.0, 0.0, 0.0, 1.0)
        specs = [
            h.length_spec(bottom, 150.0),
            h.length_spec(left, 80.0),
        ]
        H, report = h.solve_corners(
            corners, specs, line_meta=h.line_by_geo(bottom, left))
        c0, cx, c1, cy = h.corners_from_homography(H)
        width = c0.distanceToPoint(cx)
        height = c0.distanceToPoint(cy)
        self.assertAlmostEqual(width, 150.0, delta=h.LENGTH_TOL_MM)
        self.assertAlmostEqual(height, 80.0, delta=h.LENGTH_TOL_MM)

class TestAxisConstraints(unittest.TestCase):
    """Horizontal / vertical alignment on skewed quads."""

    def setUp(self):
        self.skew = h.corners_trapezoid(200.0, 170.0, 100.0, shear_top=25.0)
        # Deliberately slanted bottom edge in UV (not already axis-aligned).
        self.bottom = h.line_uv(0, 0.05, 0.02, 0.95, 0.15)
        self.left = h.line_uv(1, 0.02, 0.05, 0.08, 0.95)

    def test_horizontal_aligns_bottom_edge(self):
        H0 = h.homography_from_corners(self.skew)
        self.assertGreater(
            abs(h.axis_sin(H0, self.bottom, "horizontal")), h.SIN_TOL)

        constraints = h.empty_constraints()
        constraints["horizontal"] = [{"geo": 0}]
        specs = [h.length_spec(
            self.bottom, h.length_mm(H0, self.bottom))]
        H, report = h.solve_corners(
            self.skew, specs,
            constraints=constraints,
            line_meta=h.line_by_geo(self.bottom, self.left))
        self.assertIsNotNone(H)
        self.assertLess(
            abs(h.axis_sin(H, self.bottom, "horizontal")), h.SIN_TOL)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)

    def test_vertical_aligns_left_edge(self):
        H0 = h.homography_from_corners(self.skew)
        self.assertGreater(
            abs(h.axis_sin(H0, self.left, "vertical")), h.SIN_TOL)

        constraints = h.empty_constraints()
        constraints["vertical"] = [{"geo": 1}]
        specs = [h.length_spec(
            self.left, h.length_mm(H0, self.left))]
        H, report = h.solve_corners(
            self.skew, specs,
            constraints=constraints,
            line_meta=h.line_by_geo(self.bottom, self.left))
        self.assertIsNotNone(H)
        self.assertLess(
            abs(h.axis_sin(H, self.left, "vertical")), h.SIN_TOL)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)

    def test_horizontal_with_sketch_placement(self):
        """Axis reference follows sketch X/Y when sketch is provided."""
        import FreeCAD as App

        doc = h.new_document("AxisSketch")
        try:
            img = h.make_aligned_image(doc, self.skew)
            sketch = h.make_sketch_on_image(doc, img)
            sketch.Placement.Rotation = (
                sketch.Placement.Rotation
                * App.Rotation(App.Vector(0, 0, 1), 15.0))

            H0 = h.homography_from_corners(self.skew)
            constraints = h.empty_constraints()
            constraints["horizontal"] = [{"geo": 0}]
            specs = [h.length_spec(
                self.bottom, h.length_mm(H0, self.bottom))]
            H, report = h.solve_corners(
                self.skew, specs,
                constraints=constraints,
                line_meta=h.line_by_geo(self.bottom),
                sketch=sketch)
            self.assertIsNotNone(H)
            self.assertLess(
                abs(h.axis_sin(H, self.bottom, "horizontal", sketch=sketch)),
                h.SIN_TOL)
        finally:
            h.close_document(doc)

    def test_horizontal_and_vertical_together(self):
        """Both axis constraints satisfied on a skewed quad."""
        H0 = h.homography_from_corners(self.skew)
        self.assertGreater(
            abs(h.axis_sin(H0, self.bottom, "horizontal")), h.SIN_TOL)
        self.assertGreater(
            abs(h.axis_sin(H0, self.left, "vertical")), h.SIN_TOL)

        constraints = h.empty_constraints()
        constraints["horizontal"] = [{"geo": 0}]
        constraints["vertical"] = [{"geo": 1}]
        specs = [
            h.length_spec(self.bottom, h.length_mm(H0, self.bottom)),
            h.length_spec(self.left, h.length_mm(H0, self.left)),
        ]
        H, report = h.solve_corners(
            self.skew, specs,
            constraints=constraints,
            line_meta=h.line_by_geo(self.bottom, self.left))
        self.assertLess(
            abs(h.axis_sin(H, self.bottom, "horizontal")), h.SIN_TOL)
        self.assertLess(
            abs(h.axis_sin(H, self.left, "vertical")), h.SIN_TOL)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)

    def test_horizontal_line_lies_on_sketch_x_after_rebuild(self):
        """Horizontal (sketch +X) → rebuilt line parallel to sketch-X."""
        import FreeCAD as App

        doc = h.new_document("HorizRebuild")
        try:
            start = h.corners_trapezoid(200.0, 170.0, 100.0, shear_top=25.0)
            img = h.make_aligned_image(doc, start)
            sketch = h.make_sketch_on_image(doc, img)
            line = h.line_uv(0, 0.05, 0.02, 0.95, 0.15)
            H0 = h.homography_from_corners(start)
            constraints = h.empty_constraints()
            constraints["horizontal"] = [{"geo": 0}]
            specs = [h.length_spec(line, h.length_mm(H0, line))]
            H, report = h.solve_corners(
                start, specs,
                constraints=constraints,
                line_meta=h.line_by_geo(line),
                sketch=sketch)
            self.assertLess(
                abs(h.axis_sin(H, line, "horizontal", sketch=sketch)),
                h.SIN_TOL)

            corners = h.corners_from_homography(H)
            image_tools._restore_aligned_corners(img, corners)
            image_tools._sync_warp_from_corners(img)
            image_tools._update_sketch_from_uv_lines(
                sketch, img, [line])

            seg = sketch.Geometry[0]
            local = App.Vector(seg.EndPoint) - App.Vector(seg.StartPoint)
            self.assertGreater(local.Length, 1e-6)
            sin_xy = abs(local.y) / local.Length
            self.assertLess(sin_xy, h.SIN_TOL)
        finally:
            h.close_document(doc)

class TestAngleConstraints(unittest.TestCase):
    """Parallel and perpendicular constraints between two lines."""

    def setUp(self):
        self.skew = h.corners_parallelogram(180.0, 120.0, shear_x=35.0)
        self.line_a = h.line_uv(0, 0.15, 0.15, 0.85, 0.25)
        self.line_b = h.line_uv(1, 0.15, 0.15, 0.25, 0.85)

    def test_perpendicular_L_shape(self):
        """Two lines from a corner should become orthogonal after solve."""
        H0 = h.homography_from_corners(self.skew)
        dot0 = abs(h.dot_directions(H0, self.line_a, self.line_b))
        self.assertGreater(dot0, 0.05)

        constraints = h.empty_constraints()
        constraints["perpendicular"] = [{"geo_a": 0, "geo_b": 1}]
        specs = [
            h.length_spec(self.line_a, h.length_mm(H0, self.line_a)),
            h.length_spec(self.line_b, h.length_mm(H0, self.line_b)),
        ]
        H, report = h.solve_corners(
            self.skew, specs,
            constraints=constraints,
            line_meta=h.line_by_geo(self.line_a, self.line_b))
        self.assertIsNotNone(H)
        dot = abs(h.dot_directions(H, self.line_a, self.line_b))
        self.assertLess(dot, h.SIN_TOL)
        self.assertLess(report["length_error"], h.LENGTH_TOL_MM)

    def test_parallel_two_edges(self):
        """Bottom and top-like edges forced parallel."""
        line_top = h.line_uv(2, 0.1, 0.9, 0.9, 0.85)
        H0 = h.homography_from_corners(self.skew)
        sin0 = abs(h.sin_angle_between(H0, self.line_a, line_top))
        self.assertGreater(sin0, 0.05)

        constraints = h.empty_constraints()
        constraints["parallel"] = [{"geo_a": 0, "geo_b": 2}]
        specs = [
            h.length_spec(self.line_a, h.length_mm(H0, self.line_a)),
            h.length_spec(line_top, h.length_mm(H0, line_top)),
        ]
        H, report = h.solve_corners(
            self.skew, specs,
            constraints=constraints,
            line_meta=h.line_by_geo(self.line_a, self.line_b, line_top))
        self.assertIsNotNone(H)
        sin_par = abs(h.sin_angle_between(H, self.line_a, line_top))
        self.assertLess(sin_par, h.SIN_TOL)


class TestComputeCalibrationFromSpecs(unittest.TestCase):
    """Integration with AlignedImage + compute_calibration_from_specs."""

    def test_updates_aligned_image_corners(self):
        doc = h.new_document("ComputeSpecs")
        try:
            start = h.corners_trapezoid(210.0, 150.0, 105.0, shear_top=20.0)
            ref = h.corners_rectangle(200.0, 100.0)
            bottom = h.line_uv(0, 0.1, 0.0, 0.9, 0.0)
            left = h.line_uv(1, 0.0, 0.1, 0.0, 0.9)
            specs = h.targets_from_reference(ref, bottom, left)

            img = h.make_aligned_image(doc, start)
            constraints = image_calibration_objects.default_constraints()
            corners_new, H, opt_info, meta = (
                image_tools.compute_calibration_from_specs(
                    specs, img, constraints=constraints,
                    line_by_geo=h.line_by_geo(bottom, left)))
            image_tools._restore_aligned_corners(img, corners_new)
            image_tools._sync_warp_from_corners(img)
            self.assertTrue(meta.get("exact", False))
            for spec in specs:
                err = abs(h.length_mm(H, spec) - spec["target"])
                self.assertLess(err, h.LENGTH_TOL_MM)
        finally:
            h.close_document(doc)


class TestConstraintRemapping(unittest.TestCase):
    """Line index remapping (legacy geo ids)."""

    def test_remap_swaps_line_indices(self):
        constraints = image_calibration_objects.default_constraints()
        constraints["lengths"] = [
            {"line": 0, "target_mm": 100.0},
            {"line": 1, "target_mm": 50.0},
        ]
        constraints["horizontal"] = [{"line": 2}]
        constraints["parallel"] = [{"line_a": 0, "line_b": 3}]
        constraints["perpendicular"] = [{"line_a": 1, "line_b": 4}]

        line_map = {0: 5, 1: 4, 2: 3, 3: 2, 4: 1}
        remapped = image_tools._remap_constraint_geos(constraints, line_map)

        self.assertEqual(remapped["lengths"][0]["line"], 5)
        self.assertEqual(remapped["lengths"][1]["line"], 4)
        self.assertEqual(remapped["horizontal"][0]["line"], 3)
        self.assertEqual(remapped["parallel"][0]["line_a"], 5)
        self.assertEqual(remapped["parallel"][0]["line_b"], 2)
        self.assertEqual(remapped["perpendicular"][0]["line_a"], 4)
        self.assertEqual(remapped["perpendicular"][0]["line_b"], 1)


class TestDistortionEnergyGating(unittest.TestCase):
    """E_angle in residuals only when primary Jacobian rank < 6; Δt always in corners."""

    def test_two_lengths_underdetermined_includes_e(self):
        corners = h.corners_rectangle(100.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        left = h.line_uv(1, 0.0, 0.0, 0.0, 1.0)
        specs = [h.length_spec(bottom, 150.0), h.length_spec(left, 80.0)]
        H, report = h.solve_corners(
            corners, specs, line_meta=h.line_by_geo(bottom, left))
        self.assertIsNotNone(H)
        self.assertLess(report["constraint_rank"], 6)
        self.assertTrue(report["include_distortion_energy"])
        self.assertEqual(report["determinacy"], "unterbestimmt")

    def test_one_length_and_horizontal_underdetermined(self):
        skew = h.corners_trapezoid(200.0, 170.0, 100.0, shear_top=25.0)
        bottom = h.line_uv(0, 0.05, 0.02, 0.95, 0.15)
        H0 = h.homography_from_corners(skew)
        constraints = h.empty_constraints()
        constraints["horizontal"] = [{"geo": 0}]
        specs = [h.length_spec(bottom, h.length_mm(H0, bottom))]
        H, report = h.solve_corners(
            skew, specs,
            constraints=constraints,
            line_meta=h.line_by_geo(bottom))
        self.assertIsNotNone(H)
        self.assertLess(report["constraint_rank"], 6)
        self.assertTrue(report["include_distortion_energy"])
        self.assertEqual(report["determinacy"], "unterbestimmt")

    def test_many_length_constraints_may_exclude_side_terms(self):
        """Eight length specs can reach full rank → E_angle omitted, Δt kept."""
        from freecad.frametools import image_constraint_solver as cs

        corners = h.corners_rectangle(120.0, 90.0)
        H0 = h.homography_from_corners(corners)
        lines = [
            h.line_uv(0, 0.0, 0.0, 1.0, 0.0),
            h.line_uv(1, 0.0, 0.0, 0.0, 1.0),
            h.line_uv(2, 0.0, 0.0, 1.0, 1.0),
            h.line_uv(3, 0.2, 0.0, 0.8, 1.0),
            h.line_uv(4, 0.0, 0.2, 1.0, 0.8),
            h.line_uv(5, 0.1, 0.1, 0.9, 0.5),
            h.line_uv(6, 0.0, 0.5, 1.0, 0.5),
            h.line_uv(7, 0.5, 0.0, 0.5, 1.0),
        ]
        specs = [h.length_spec(line, h.length_mm(H0, line)) for line in lines]
        params0 = image_tools._pack_corners_xy(corners)
        z_vals = image_tools._corner_z_values(corners)
        line_by_geo = h.line_by_geo(*lines)
        rank, n_primary, include_angle = image_tools._primary_constraint_rank(
            params0, specs, z_vals, line_by_geo=line_by_geo)
        self.assertEqual(n_primary, 8)
        if rank >= 6:
            self.assertFalse(include_angle)
        else:
            self.assertTrue(include_angle)
        r_with = cs._calibration_residuals(
            params0, specs, params0, z_vals, include_angle_energy=include_angle)
        r_no_trans = cs._calibration_residuals(
            params0, specs, params0, z_vals,
            include_angle_energy=include_angle, include_centroid=False)
        self.assertEqual(len(r_with), len(r_no_trans) + 1)


class TestFixedPointConstraints(unittest.TestCase):
    """Fixed sketch nodes Pk with target world XY."""

    def test_fixed_point_holds_target(self):
        corners = h.corners_rectangle(200.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        H0 = h.homography_from_corners(corners)
        p0 = h.corners_from_homography(H0)[0]
        pt = h.point_uv(0, 0.0, 0.0, p0.x, p0.y)
        constraints = h.empty_constraints()
        constraints["fixed_points"] = [{
            "point": 0,
            "target_x_mm": float(p0.x),
            "target_y_mm": float(p0.y),
        }]
        specs = [h.length_spec(bottom, h.length_mm(H0, bottom))]
        H, report = h.solve_corners(
            corners, specs, constraints=constraints,
            line_meta=h.line_by_geo(bottom),
            point_meta=h.point_by_index(pt))
        pos = image_tools._apply_homography_uv(0.0, 0.0, H)
        self.assertLess(abs(pos.x - p0.x), h.LENGTH_TOL_MM)
        self.assertLess(abs(pos.y - p0.y), h.LENGTH_TOL_MM)
        self.assertFalse(report["include_translation_side"])
        self.assertEqual(report["mode"], "uniform_scale")

    def test_centroid_skipped_with_fixed_point(self):
        corners = h.corners_rectangle(200.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        H0 = h.homography_from_corners(corners)
        p0 = h.corners_from_homography(H0)[0]
        pt = h.point_uv(0, 0.0, 0.0, p0.x, p0.y)
        constraints = h.empty_constraints()
        constraints["fixed_points"] = [{
            "point": 0,
            "target_x_mm": float(p0.x),
            "target_y_mm": float(p0.y),
        }]
        specs = [h.length_spec(bottom, h.length_mm(H0, bottom))]
        _, report = h.solve_corners(
            corners, specs, constraints=constraints,
            line_meta=h.line_by_geo(bottom),
            point_meta=h.point_by_index(pt))
        self.assertFalse(report["include_translation_side"])

    def test_fixed_point_moves_quad(self):
        corners = h.corners_rectangle(200.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        H0 = h.homography_from_corners(corners)
        p0 = h.corners_from_homography(H0)[0]
        target_x = float(p0.x) + 15.0
        target_y = float(p0.y) + 5.0
        pt = h.point_uv(0, 0.0, 0.0, p0.x, p0.y)
        constraints = h.empty_constraints()
        constraints["fixed_points"] = [{
            "point": 0,
            "target_x_mm": target_x,
            "target_y_mm": target_y,
        }]
        specs = [h.length_spec(bottom, h.length_mm(H0, bottom))]
        H, report = h.solve_corners(
            corners, specs, constraints=constraints,
            line_meta=h.line_by_geo(bottom),
            point_meta=h.point_by_index(pt))
        pos = image_tools._apply_homography_uv(0.0, 0.0, H)
        self.assertLess(abs(pos.x - target_x), h.LENGTH_TOL_MM)
        self.assertLess(abs(pos.y - target_y), h.LENGTH_TOL_MM)
        self.assertIn(report["mode"], ("uniform_scale", "uv_scale", "corners"))

    def test_fixed_point_two_lengths_resolved(self):
        """Two lengths + fixed point: scale phases include fixed-point residuals."""
        corners = h.corners_rectangle(200.0, 100.0)
        bottom = h.line_uv(0, 0.0, 0.0, 1.0, 0.0)
        left = h.line_uv(1, 0.0, 0.0, 0.0, 1.0)
        H0 = h.homography_from_corners(corners)
        p0 = h.corners_from_homography(H0)[0]
        target_x = float(p0.x) + 10.0
        target_y = float(p0.y)
        pt = h.point_uv(0, 0.0, 0.0, p0.x, p0.y)
        constraints = h.empty_constraints()
        constraints["fixed_points"] = [{
            "point": 0,
            "target_x_mm": target_x,
            "target_y_mm": target_y,
        }]
        specs = [
            h.length_spec(bottom, h.length_mm(H0, bottom)),
            h.length_spec(left, h.length_mm(H0, left)),
        ]
        H, report = h.solve_corners(
            corners, specs, constraints=constraints,
            line_meta=h.line_by_geo(bottom, left),
            point_meta=h.point_by_index(pt))
        pos = image_tools._apply_homography_uv(0.0, 0.0, H)
        self.assertLess(abs(pos.x - target_x), h.LENGTH_TOL_MM)
        self.assertLess(abs(pos.y - target_y), h.LENGTH_TOL_MM)
        for spec in specs:
            err = abs(h.length_mm(H, spec) - spec["target"])
            self.assertLess(err, h.LENGTH_TOL_MM)
        self.assertIn(
            report["mode"], ("uniform_scale", "uv_scale", "corners"))
        self.assertFalse(report["include_translation_side"])


class TestPointsMeta(unittest.TestCase):
    def test_welded_endpoints_one_point(self):
        lines = [
            {
                "u0": 0.1, "v0": 0.2, "u1": 0.5, "v1": 0.2,
                "w0": h.vec(0, 0), "w1": h.vec(10, 0),
            },
            {
                "u0": 0.1, "v0": 0.2, "u1": 0.1, "v1": 0.8,
                "w0": h.vec(0, 0), "w1": h.vec(0, 10),
            },
        ]
        points = image_tools._points_meta_from_lines_meta(lines)
        self.assertEqual(len(points), 3)
        shared = [
            pt for pt in points
            if abs(pt["u"] - 0.1) < 1e-9 and abs(pt["v"] - 0.2) < 1e-9]
        self.assertEqual(len(shared), 1)


if __name__ == "__main__":
    unittest.main()
