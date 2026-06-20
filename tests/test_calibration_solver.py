"""Tests for image calibration homography / corner solver."""

import math
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
    """One length constraint → uniform scaling about the image origin."""

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


class TestTwoLengthScale(unittest.TestCase):
    """Two length constraints on perpendicular edges → anisotropic 2D scale."""

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
        self.assertGreater(report["distortion_energy"], h.DISTORTION_TOL)

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


if __name__ == "__main__":
    unittest.main()
