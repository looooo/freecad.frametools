"""Synthetic geometry for calibration solver tests."""

import math

import FreeCAD as App
import numpy as np

from freecad.frametools import image_calibration_objects
from freecad.frametools import image_objects
from freecad.frametools import image_tools


LENGTH_TOL_MM = 0.05
ANGLE_TOL_DEG = 1.0
SIN_TOL = float(np.sin(np.deg2rad(ANGLE_TOL_DEG)))


def vec(x, y, z=0.0):
    return App.Vector(float(x), float(y), float(z))


def corners_rectangle(w, h, origin=(0.0, 0.0, 0.0)):
    """UV unit square mapped to axis-aligned rectangle w × h."""
    ox, oy, oz = origin
    c0 = vec(ox, oy, oz)
    cx = vec(ox + w, oy, oz)
    cy = vec(ox, oy + h, oz)
    c1 = vec(ox + w, oy + h, oz)
    return c0, cx, c1, cy


def corners_trapezoid(w_bottom, w_top, h, shear_top=0.0):
    """Trapezoid: bottom w_bottom, top w_top, height h (perspective-like)."""
    c0 = vec(0, 0, 0)
    cx = vec(w_bottom, 0, 0)
    cy = vec(shear_top, h, 0)
    c1 = vec(shear_top + w_top, h, 0)
    return c0, cx, c1, cy


def corners_parallelogram(w, h, shear_x=0.0):
    """Parallelogram with horizontal base length w and height h."""
    c0 = vec(0, 0, 0)
    cx = vec(w, 0, 0)
    cy = vec(shear_x, h, 0)
    c1 = vec(shear_x + w, h, 0)
    return c0, cx, c1, cy


def corners_rotated_rectangle(w, h, angle_deg, origin=(0.0, 0.0, 0.0)):
    """Rectangle rotated in the XY plane."""
    ox, oy, oz = origin
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    c0 = vec(ox, oy, oz)
    cx = vec(ox + w * cos_a, oy + w * sin_a, oz)
    cy = vec(ox - h * sin_a, oy + h * cos_a, oz)
    c1 = vec(
        ox + w * cos_a - h * sin_a,
        oy + w * sin_a + h * cos_a,
        oz)
    return c0, cx, c1, cy


def homography_from_corners(corners):
    return image_tools._homography_from_corners(*corners)


def line_uv(geo, u0, v0, u1, v1):
    return {
        "geo": int(geo),
        "label": "L{}".format(geo),
        "u0": float(u0),
        "v0": float(v0),
        "u1": float(u1),
        "v1": float(v1),
    }


def length_mm(H, line):
    return image_tools._line_length_uv(
        line["u0"], line["v0"], line["u1"], line["v1"], H)


def direction_xy(H, line):
    return image_tools._direction_xy_from_uv_line(
        line["u0"], line["v0"], line["u1"], line["v1"], H)


def sin_angle_between(H, line_a, line_b):
    da = direction_xy(H, line_a)
    db = direction_xy(H, line_b)
    return float(da[0] * db[1] - da[1] * db[0])


def dot_directions(H, line_a, line_b):
    da = direction_xy(H, line_a)
    db = direction_xy(H, line_b)
    return float(np.dot(da, db))


def axis_sin(H, line, axis, sketch=None):
    """axis: 'u' (horizontal / sketch-X) or 'v' (vertical / sketch-Y)."""
    if axis == "horizontal":
        axis = "u"
    elif axis == "vertical":
        axis = "v"
    return image_tools._axis_alignment_sin(line, H, axis, sketch)


def length_spec(line, target_mm):
    return {
        "label": line.get("label", "L{}".format(line["geo"])),
        "u0": line["u0"],
        "v0": line["v0"],
        "u1": line["u1"],
        "v1": line["v1"],
        "target": float(target_mm),
    }


def line_by_geo(*lines):
    return {int(line["geo"]): line for line in lines}


def empty_constraints():
    return image_calibration_objects.default_constraints()


def solve_corners(corners, specs, constraints=None, line_meta=None, sketch=None):
    """Run corner calibration for synthetic geometry."""
    if constraints is None:
        constraints = empty_constraints()
    if line_meta is None:
        line_meta = line_by_geo()
    params0 = image_tools._pack_corners_xy(corners)
    z_vals = image_tools._corner_z_values(corners)
    _, H, opt_info, meta = image_tools._solve_corner_calibration(
        specs, params0, z_vals,
        constraints=constraints,
        line_by_geo=line_meta,
        sketch=sketch)
    residuals = opt_info.get("residuals") or []
    max_len_err = max((abs(r) for r in residuals), default=0.0)
    report = {
        "length_error": max_len_err,
        "success": opt_info.get("success"),
        "exact": meta.get("exact"),
    }
    return H, report


def new_document(name="TestCalibration"):
    doc = App.newDocument(name)
    return doc


def close_document(doc):
    if doc is not None:
        App.closeDocument(doc.Name)


def make_aligned_image(doc, corners, name="AlignedImage"):
    obj = doc.addObject("App::FeaturePython", name)
    image_objects.AlignedImage(obj)
    c0, cx, c1, cy = corners
    obj.Corner0 = App.Vector(c0)
    obj.CornerX = App.Vector(cx)
    obj.Corner1 = App.Vector(c1)
    obj.CornerY = App.Vector(cy)
    image_tools._sync_warp_from_corners(obj)
    return obj


def make_sketch_on_image(doc, img, name="CalibSketch"):
    sketch = doc.addObject("Sketcher::SketchObject", name)
    sketch.Placement = image_tools._sketch_placement_from_image(img)
    return sketch


def targets_from_reference(corners_ref, *lines):
    """Build length specs from a reference rectangle / corner pose."""
    H = homography_from_corners(corners_ref)
    return [length_spec(line, length_mm(H, line)) for line in lines]
