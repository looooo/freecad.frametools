"""Print solver diagnostics like solve_image_calibration (freecadcmd).

Usage:
  pixi run freecadcmd tests/debug_solver_report.py
  pixi run freecadcmd tests/debug_solver_report.py align_image_test_1.json
"""

import os
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if root not in sys.path:
    sys.path.insert(0, root)

from freecad.frametools import image_constraint_solver as cs
from freecad.frametools import image_tools

from tests import helpers as h


def run_fixture(name="align_image_test_1.json"):
    data = h.load_fixture(name)
    corners = h.corners_from_fixture(data)
    specs, line_by_geo = h.length_specs_from_fixture(data)
    constraints = data["constraints"]

    params0 = image_tools._pack_corners_xy(corners)
    z_vals = image_tools._corner_z_values(corners)
    _, H, opt_info, meta = image_tools._solve_corner_calibration(
        specs, params0, z_vals,
        constraints=constraints,
        line_by_geo=line_by_geo)

    cs._print_calibration_constraint_report(
        constraints, line_by_geo, H, specs,
        meta=meta, opt_info=opt_info)
    return meta, opt_info


def main():
    name = "align_image_test_1.json"
    for arg in sys.argv[1:]:
        if arg.endswith(".json"):
            name = os.path.basename(arg)
            break
    print("Fixture: tests/fixtures/{}\n".format(name))
    run_fixture(name)


main()
