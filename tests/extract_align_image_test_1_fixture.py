"""Extract calibration fixture from example/align_image_test_1.FCStd.

Run: pixi run freecadcmd tests/extract_align_image_test_1_fixture.py
Writes: tests/fixtures/align_image_test_1.json
"""

import json
import os
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if root not in sys.path:
    sys.path.insert(0, root)

import FreeCAD as App

from freecad.frametools import image_calibration_objects
from freecad.frametools import image_constraint_solver as cs
from freecad.frametools import image_point_alignment as pa
from freecad.frametools import image_homography as hg
from freecad.frametools import image_tools


def _vec_tuple(v):
    return [float(v.x), float(v.y), float(v.z)]


def _line_dict(line):
    return {
        "geo": int(line.get("geo", line.get("line", 0))),
        "label": line.get("label", "L{}".format(line.get("line", 0))),
        "u0": float(line["u0"]),
        "v0": float(line["v0"]),
        "u1": float(line["u1"]),
        "v1": float(line["v1"]),
    }


def main():
    fcstd = os.path.join(root, "example", "align_image_test_1.FCStd")
    if not os.path.isfile(fcstd):
        raise SystemExit("Missing {}".format(fcstd))

    doc = App.openDocument(fcstd)
    try:
        cal = doc.getObject("ImageCalibration001")
        if cal is None:
            cal = doc.getObject("ImageCalibration")
        if cal is None:
            raise RuntimeError("ImageCalibration object not found")

        constraints = image_calibration_objects.parse_constraints(
            cal.Constraints)
        plane = cal.Image
        corners0 = pa._corners_from_image_plane(plane)[:4]

        stored_lines = image_calibration_objects.parse_lines(cal.Lines)
        lines_meta = []
        for i, item in enumerate(stored_lines):
            lines_meta.append({
                "line": int(item.get("line", i)),
                "geo": int(item.get("line", i)),
                "label": "L{}".format(i),
                "u0": float(item["u0"]),
                "v0": float(item["v0"]),
                "u1": float(item["u1"]),
                "v1": float(item["v1"]),
            })
        line_by_geo = cs._line_by_index_from_lines_meta(lines_meta)
        length_specs = cs._length_specs_from_constraints(
            constraints, line_by_geo)

        params0 = hg._pack_corners_xy(corners0)
        z_vals = hg._corner_z_values(corners0)
        corners_new, H, opt_info, meta = image_tools._solve_corner_calibration(
            length_specs, params0, z_vals,
            constraints=constraints,
            line_by_geo=line_by_geo,
            sketch=cal.Sketch)

        angle_meta = cs._angle_energy_report(
            hg._pack_corners_xy(corners_new),
            hg._pack_corners_xy(corners0),
            hg._corner_z_values(corners0))

        fixture = {
            "source": "example/align_image_test_1.FCStd",
            "description": (
                "Two length constraints (400 mm, 1200.499 mm) on sketch "
                "lines L0/L1 of a photo ImagePlane; no angle constraints."
            ),
            "corners0": {
                "c0": _vec_tuple(corners0[0]),
                "cx": _vec_tuple(corners0[1]),
                "c1": _vec_tuple(corners0[2]),
                "cy": _vec_tuple(corners0[3]),
            },
            "lines": [_line_dict(line) for line in lines_meta],
            "constraints": constraints,
            "length_targets_mm": [
                float(item["target_mm"]) for item in constraints["lengths"]
            ],
            "solver_at_extract": {
                "success": bool(opt_info.get("success")),
                "exact": bool(meta.get("exact")),
                "mode": meta.get("mode"),
                "constraint_rank": meta.get("constraint_rank"),
                "include_distortion_energy": meta.get(
                    "include_distortion_energy"),
                "determinacy": meta.get("determinacy"),
                "angle_preserving_energy": float(
                    angle_meta["angle_preserving_energy"]),
                "length_errors_mm": [
                    abs(float(r)) for r in (opt_info.get("residuals") or [])
                ],
            },
        }

        out_dir = os.path.join(root, "tests", "fixtures")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "align_image_test_1.json")
        with open(out_path, "w", encoding="utf-8") as fh:
            json.dump(fixture, fh, indent=2)
            fh.write("\n")
        print("Wrote {}".format(out_path))
    finally:
        App.closeDocument(doc.Name)


if __name__ == "__main__":
    main()

main()
