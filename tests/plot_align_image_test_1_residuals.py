"""Trace and plot calibration residuals for align_image_test_1 fixture.

Run:
  pixi run freecadcmd tests/plot_align_image_test_1_residuals.py

Writes under tests/output/:
  align_image_test_1_one_length_residuals.{csv,png}
  align_image_test_1_two_lengths_residuals.{csv,png}
  align_image_test_1_two_lengths_cold_start.{csv,png}  (legacy 8-DOF start)
"""

import csv
import os
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if root not in sys.path:
    sys.path.insert(0, root)

import numpy as np
from scipy.optimize import least_squares

import FreeCAD as App

from freecad.frametools import image_constraint_solver as cs
from freecad.frametools import image_homography as hg

from tests import helpers as h

_CORNER_LABELS = cs._CORNER_ANGLE_LABELS


def _decompose_residuals(
        params, specs, params0, z_vals, include_translation,
        constraints=None, line_by_geo=None, sketch=None):
    H = hg._homography_from_xy_params(params, z_vals)
    length_mm = np.asarray(
        cs._length_residuals_for_specs(specs, H), dtype=float)
    n = len(specs)
    length_scaled = length_mm / cs._CALIB_LENGTH_TOLERANCE_MM
    corner_e = cs._angle_preserving_energy_per_corner(params, params0, z_vals)
    angle_sqrt = cs._angle_energy_side_residuals(params, params0, z_vals)
    rigid = cs._rigid_motion_stats(params, params0, z_vals)
    trans_scaled = rigid["translation_mm"] / cs._CALIB_RIGID_TRANSLATION_TOLERANCE_MM
    full = cs._calibration_residuals(
        params, specs, params0, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
        include_side_terms=include_translation)
    return {
        "length_mm": length_mm,
        "length_scaled": length_scaled,
        "corner_energy": corner_e,
        "angle_sqrt": angle_sqrt,
        "trans_mm": rigid["translation_mm"],
        "trans_scaled": trans_scaled,
        "full": full,
        "cost_half": 0.5 * float(np.dot(full, full)),
        "n_lengths": n,
    }


def _pad_length_row(length_mm, length_scaled, n_cols=2):
    mm = [0.0] * n_cols
    sc = [0.0] * n_cols
    for i in range(min(n_cols, len(length_mm))):
        mm[i] = float(length_mm[i])
        sc[i] = float(length_scaled[i])
    return mm, sc


def _write_csv(path, history, phase_at=None):
    """phase_at: optional list of (step_index, label) vertical markers metadata."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow([
            "step", "phase", "cost_half", "cost_length", "cost_angle", "cost_trans",
            "len0_mm", "len1_mm", "len0_scaled", "len1_scaled",
            "E_c0", "E_cx", "E_c1", "E_cy",
            "sqrt_E_c0", "sqrt_E_cx", "sqrt_E_c1", "sqrt_E_cy",
            "E_sum", "trans_mm", "trans_scaled",
        ])
        for i, dec in enumerate(history):
            e = dec["corner_energy"]
            s = dec["angle_sqrt"]
            ls = dec["length_scaled"]
            lm = dec["length_mm"]
            mm, sc = _pad_length_row(lm, ls)
            cost_len = 0.5 * float(np.dot(np.asarray(ls, dtype=float), ls))
            cost_ang = 0.5 * float(np.dot(s, s))
            cost_tr = 0.5 * dec["trans_scaled"] ** 2
            phase = dec.get("phase", "")
            w.writerow([
                i, phase,
                dec["cost_half"], cost_len, cost_ang, cost_tr,
                mm[0], mm[1], sc[0], sc[1],
                e[0], e[1], e[2], e[3],
                s[0], s[1], s[2], s[3],
                sum(e),
                dec["trans_mm"], dec["trans_scaled"],
            ])


def _try_plot(csv_path, png_path, title, phase_markers=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        App.Console.PrintWarning(
            "matplotlib nicht verfügbar — nur CSV: {}\n".format(csv_path))
        return False

    steps, cost, cost_len, cost_ang, cost_tr = [], [], [], [], []
    len0s, len1s, e_sum = [], [], []
    sqrt_e = {k: [] for k in _CORNER_LABELS}
    has_len1 = False

    with open(csv_path, encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            steps.append(int(row["step"]))
            cost.append(float(row["cost_half"]))
            cost_len.append(float(row["cost_length"]))
            cost_ang.append(float(row["cost_angle"]))
            cost_tr.append(float(row["cost_trans"]))
            len0s.append(abs(float(row["len0_mm"])))
            l1 = abs(float(row["len1_mm"]))
            len1s.append(l1)
            if l1 > 1e-15:
                has_len1 = True
            e_sum.append(float(row["E_sum"]))
            for k, col in zip(
                    _CORNER_LABELS,
                    ("sqrt_E_c0", "sqrt_E_cx", "sqrt_E_c1", "sqrt_E_cy")):
                sqrt_e[k].append(float(row[col]))

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    fig.suptitle(title)

    axes[0].plot(steps, cost, "k-", lw=1.2, label="½‖r‖² gesamt")
    axes[0].plot(steps, cost_len, "C0--", alpha=0.8, label="Längen-Anteil")
    axes[0].plot(steps, cost_ang, "C3--", alpha=0.8, label="E_angle-Anteil")
    axes[0].set_ylabel("Kosten")
    axes[0].legend(loc="upper right", fontsize=7)
    axes[0].grid(True, alpha=0.3)
    if max(cost) > 0:
        axes[0].set_yscale("log")

    axes[1].plot(steps, cost_len, label="Längen")
    axes[1].plot(steps, cost_ang, label="E_angle")
    axes[1].plot(steps, cost_tr, label="Translation", alpha=0.7)
    axes[1].set_ylabel("Kostenanteile (linear)")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(steps, len0s, label="|ΔL0| mm")
    if has_len1:
        axes[2].plot(steps, len1s, label="|ΔL1| mm")
    axes[2].axhline(
        cs._CALIB_LENGTH_TOLERANCE_MM, color="gray", ls="--",
        label="τ_L = {:.3f} mm".format(cs._CALIB_LENGTH_TOLERANCE_MM))
    axes[2].set_ylabel("Längenfehler [mm]")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)
    if max(len0s + len1s) > 0:
        axes[2].set_yscale("log")

    axes[3].plot(steps, e_sum, "C3-", lw=1.5, label="Σ E_angle")
    for k in _CORNER_LABELS:
        axes[3].plot(steps, sqrt_e[k], ls="--", alpha=0.8, label="√E_{}".format(k))
    axes[3].set_xlabel("Funktionsauswertung (Schritt)")
    axes[3].set_ylabel("Winkel-Energie / Residuum")
    axes[3].legend(loc="upper right", fontsize=7, ncol=2)
    axes[3].grid(True, alpha=0.3)

    if phase_markers:
        for step, label in phase_markers:
            for ax in axes:
                ax.axvline(step, color="green", alpha=0.35, lw=1)
            axes[0].annotate(
                label, xy=(step, cost[min(step, len(cost) - 1)]),
                xytext=(step + max(1, len(steps) // 20), max(cost_ang) * 1.5),
                fontsize=7, color="green")

    os.makedirs(os.path.dirname(png_path), exist_ok=True)
    fig.tight_layout()
    fig.savefig(png_path, dpi=140)
    plt.close(fig)
    return True


def _trace_uniform_scale(params0, z_vals, specs):
    """Production path: 1 length → analytic uniform scale (start + end)."""
    history = []
    dec0 = _decompose_residuals(
        params0, specs, params0, z_vals, include_translation=False)
    dec0["phase"] = "start"
    history.append(dec0)

    spec = specs[0]
    corners0 = hg._corners_from_xy_params(params0, z_vals)
    H0 = hg._homography_from_corners(*corners0)
    current = hg._line_length_uv(
        spec["u0"], spec["v0"], spec["u1"], spec["v1"], H0)
    scale = float(spec["target"]) / current
    params1 = cs._uniform_scale_params(params0, scale)
    dec1 = _decompose_residuals(
        params1, specs, params0, z_vals, include_translation=False)
    dec1["phase"] = "uniform_scale"
    history.append(dec1)
    return history, scale


def _trace_uv_scale_phase(params0, z_vals, specs):
    history = []

    def residual_fn(scales):
        params = cs._uv_scale_params(params0, scales[0], scales[1])
        dec = _decompose_residuals(
            params, specs, params0, z_vals, include_translation=True)
        dec["phase"] = "uv_scale"
        history.append(dec)
        return cs._uv_scale_length_residuals(scales, specs, params0, z_vals)

    result = least_squares(
        residual_fn,
        np.array([1.0, 1.0], dtype=float),
        ftol=cs._CALIB_SOLVER_FTOL,
        xtol=cs._CALIB_SOLVER_XTOL,
        gtol=cs._CALIB_SOLVER_GTOL,
        max_nfev=cs._CALIB_SOLVER_MAX_NFEV)
    params = cs._uv_scale_params(params0, result.x[0], result.x[1])
    return result, history, params


def _trace_corner_phase(
        params_start, params0, z_vals, specs,
        constraints=None, line_by_geo=None, sketch=None, phase_label="corners"):
    rank, n_primary, include_translation = cs._primary_constraint_rank(
        params0, specs, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch)
    history = []

    def residual_fn(params):
        dec = _decompose_residuals(
            params, specs, params0, z_vals, include_translation,
            constraints=constraints, line_by_geo=line_by_geo, sketch=sketch)
        dec["phase"] = phase_label
        history.append(dec)
        return cs._calibration_residuals(
            params, specs, params0, z_vals,
            constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
            include_side_terms=include_translation)

    result = least_squares(
        residual_fn,
        np.asarray(params_start, dtype=float).copy(),
        ftol=cs._CALIB_SOLVER_FTOL,
        xtol=cs._CALIB_SOLVER_XTOL,
        gtol=cs._CALIB_SOLVER_GTOL,
        max_nfev=cs._CALIB_SOLVER_MAX_NFEV)
    return result, history, rank, include_translation


def _merge_histories(*parts):
    out = []
    for part in parts:
        out.extend(part)
    return out


def _print_summary(label, history, extra=""):
    dec = history[-1]
    lm = dec["length_mm"]
    parts = ["|ΔL{}|={:.6f} mm".format(i, abs(lm[i])) for i in range(len(lm))]
    App.Console.PrintMessage(
        "{}: steps={} cost={:.4e} {} E_sum={:.4e}{}\n".format(
            label, len(history), dec["cost_half"],
            " ".join(parts), sum(dec["corner_energy"]), extra))


def plot_one_length(out_dir, params0, z_vals, spec, constraints):
    App.Console.PrintMessage("\n--- 1 Länge (L0 = {:.3f} mm) ---\n".format(
        spec["target"]))
    history, scale = _trace_uniform_scale(params0, z_vals, [spec])
    _print_summary("uniform_scale", history, " scale={:.6f}".format(scale))

    base = os.path.join(out_dir, "align_image_test_1_one_length_residuals")
    _write_csv(base + ".csv", history)
    if _try_plot(
            base + ".csv", base + ".png",
            "align_image_test_1 — 1 Länge (uniform_scale)"):
        App.Console.PrintMessage("PNG: {}.png\n".format(base))


def plot_two_lengths_production(out_dir, params0, z_vals, specs, constraints):
    App.Console.PrintMessage("\n--- 2 Längen (Produktions-Solver) ---\n")
    history = []
    dec0 = _decompose_residuals(
        params0, specs, params0, z_vals, include_translation=False)
    dec0["phase"] = "start"
    history.append(dec0)

    params_curr = np.asarray(params0, dtype=float).copy()
    params_curr, _uni = cs._apply_uniform_scale_phase(specs, params_curr, z_vals)
    dec1 = _decompose_residuals(
        params_curr, specs, params0, z_vals, include_translation=True)
    dec1["phase"] = "uniform_scale"
    history.append(dec1)

    if cs._scale_phase_converged(specs, params_curr, params0, z_vals):
        base = os.path.join(out_dir, "align_image_test_1_two_lengths_residuals")
        _write_csv(base + ".csv", history)
        _print_summary("uniform_scale", history)
        if _try_plot(
                base + ".csv", base + ".png",
                "align_image_test_1 — 2 Längen (Abbruch Phase 1)"):
            App.Console.PrintMessage("PNG: {}.png\n".format(base))
        return

    uv_res, uv_hist, params_uv = _trace_uv_scale_phase(
        params_curr, z_vals, specs)
    history = _merge_histories(history, uv_hist)

    if cs._scale_phase_converged(specs, params_uv, params0, z_vals):
        base = os.path.join(out_dir, "align_image_test_1_two_lengths_residuals")
        _write_csv(base + ".csv", history)
        _print_summary("uv_scale", history,
                       " sx={:.4f} sy={:.4f}".format(uv_res.x[0], uv_res.x[1]))
        if _try_plot(
                base + ".csv", base + ".png",
                "align_image_test_1 — 2 Längen (Abbruch Phase 2)"):
            App.Console.PrintMessage("PNG: {}.png\n".format(base))
        return

    cr_res, cr_hist, rank, _ = _trace_corner_phase(
        params_uv, params0, z_vals, specs,
        constraints=constraints, phase_label="corners")
    history = _merge_histories(history, cr_hist)
    phase_markers = [(len(history) - len(cr_hist), "Phase 3: Ecken")]
    _print_summary(
        "uv_scale", uv_hist,
        " sx={:.4f} sy={:.4f} success={}".format(
            uv_res.x[0], uv_res.x[1], uv_res.success))
    _print_summary(
        "corners", cr_hist,
        " success={} nfev={}".format(cr_res.success, cr_res.nfev))

    base = os.path.join(out_dir, "align_image_test_1_two_lengths_residuals")
    _write_csv(base + ".csv", history)
    if _try_plot(
            base + ".csv", base + ".png",
            "align_image_test_1 — 2 Längen (1D→2D→Ecken)",
            phase_markers=phase_markers):
        App.Console.PrintMessage("PNG: {}.png\n".format(base))


def plot_two_lengths_cold_start(out_dir, params0, z_vals, specs):
    App.Console.PrintMessage("\n--- 2 Längen (kalter Start p0, ohne UV) ---\n")
    cr_res, cr_hist, _, _ = _trace_corner_phase(
        params0, params0, z_vals, specs,
        phase_label="corners_cold")
    _print_summary(
        "cold start", cr_hist,
        " success={} nfev={}".format(cr_res.success, cr_res.nfev))

    base = os.path.join(out_dir, "align_image_test_1_two_lengths_cold_start")
    _write_csv(base + ".csv", cr_hist)
    if _try_plot(
            base + ".csv", base + ".png",
            "align_image_test_1 — 2 Längen (kalter 8-DOF-Start, legacy)"):
        App.Console.PrintMessage("PNG: {}.png\n".format(base))


def main():
    data = h.load_fixture("align_image_test_1.json")
    corners = h.corners_from_fixture(data)
    specs, line_by_geo = h.length_specs_from_fixture(data)
    spec_one, constraints_one, _ = h.single_length_fixture(data, 0)
    constraints_two = data["constraints"]

    params0 = hg._pack_corners_xy(corners)
    z_vals = hg._corner_z_values(corners)
    out_dir = os.path.join(root, "tests", "output")

    App.Console.PrintMessage(
        "\n=== Residual-Plots align_image_test_1 ===\n"
        "Ausgabe: tests/output/\n")

    plot_one_length(out_dir, params0, z_vals, spec_one, constraints_one)
    plot_two_lengths_production(
        out_dir, params0, z_vals, specs, constraints_two)
    plot_two_lengths_cold_start(out_dir, params0, z_vals, specs)


main()
