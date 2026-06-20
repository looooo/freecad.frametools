"""Quick solver benchmark (freecadcmd tests/bench_solver.py)."""

import time

import FreeCAD as App
from scipy.optimize import least_squares

from freecad.frametools import image_tools as it


def make_case():
    c0 = App.Vector(0, 0, 0)
    cx = App.Vector(220, 0, 0)
    cy = App.Vector(30, 110, 0)
    c1 = App.Vector(190, 110, 0)
    params0 = it._pack_corners_xy((c0, cx, c1, cy))
    z_vals = it._corner_z_values((c0, cx, c1, cy))
    specs = [
        {"u0": 0.1, "v0": 0.0, "u1": 0.9, "v1": 0.0, "target": 180.0, "label": "L0"},
        {"u0": 0.0, "v0": 0.1, "u1": 0.0, "v1": 0.9, "target": 90.0, "label": "L1"},
    ]
    return params0, z_vals, specs


def run(label, params0, z_vals, specs, **kw):
    t0 = time.perf_counter()
    r = least_squares(
        lambda p: it._calibration_residuals(p, specs, params0, z_vals),
        params0.copy(),
        **kw)
    dt = time.perf_counter() - t0
    corners = it._corners_from_xy_params(r.x, z_vals)
    H = it._homography_from_corners(*corners)
    errs = it._length_residuals_for_specs(specs, H)
    max_err = max(abs(e) for e in errs) if errs else 0.0
    print(
        "{:20s} {:6.0f} ms  nfev={:4d}  max_err={:.4f} mm".format(
            label, dt * 1000, r.nfev, max_err))


def main():
    params0, z_vals, specs = make_case()
    run("tight 1e-12", params0, z_vals, specs,
        ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=5000)
    run("medium 1e-8", params0, z_vals, specs,
        ftol=1e-8, xtol=1e-8, gtol=1e-8, max_nfev=500)
    run("loose 1e-6", params0, z_vals, specs,
        ftol=1e-6, xtol=1e-6, gtol=1e-6, max_nfev=200)


main()
