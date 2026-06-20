"""Geometric constraint solver."""

import FreeCAD as App
import numpy as np

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None

import Part

from . import image_calibration_objects
from . import image_objects
from . import image_homography as hg
from . import image_point_alignment as pa

REF_LINE_ENDPOINT_SNAP_MM = 5.0

_CALIB_LENGTH_TOLERANCE_MM = 0.01
_CALIB_POINT_TOLERANCE_MM = 0.01
_CALIB_RIGID_TRANSLATION_TOLERANCE_MM = 1.0
_CALIB_ANGLE_TOLERANCE_RAD = np.sin(np.deg2rad(1.0))
_CALIB_ANGLE_WEIGHT = 25.0
# Sketch angle constraints and E_angle side terms share this weight.
_CALIB_AXIS_ALIGNMENT_WEIGHT = _CALIB_ANGLE_WEIGHT
_CALIB_SOLVER_FTOL = 1e-6
_CALIB_SOLVER_XTOL = 1e-6
_CALIB_SOLVER_GTOL = 1e-6
_CALIB_SOLVER_MAX_NFEV = 250
_CALIB_SOLVER_REFINE_FTOL = 1e-9
_CALIB_SOLVER_REFINE_MAX_NFEV = 100
_CORNER_PARAM_DOF = 8
# Centroid translation penalty fixes 2 DOF in the tie-breaker → 6 effective DOF.
_CORNER_EFFECTIVE_DOF = _CORNER_PARAM_DOF - 2
_JACOBIAN_RANK_RTOL = 1e-8
_JACOBIAN_RANK_ATOL = 1e-10
_JACOBIAN_FD_EPS = 1e-7


def _has_angle_constraints(constraints):
    if not constraints:
        return False
    for key in ("parallel", "perpendicular", "horizontal", "vertical"):
        if constraints.get(key):
            return True
    return False


def _sketch_axis_directions_xy(sketch):
    rot = sketch.Placement.Rotation
    x = rot.multVec(App.Vector(1, 0, 0))
    y = rot.multVec(App.Vector(0, 1, 0))

    def xy_unit(v):
        n = np.hypot(v.x, v.y)
        if n < 1e-12:
            return np.array([1.0, 0.0])
        return np.array([v.x / n, v.y / n])

    return xy_unit(x), xy_unit(y)


def _axis_alignment_sin(line, H, axis, sketch=None):
    """Sin(angle) between a line (world XY) and a reference axis.

    axis \"u\" / horizontal: parallel to sketch +X when *sketch* is given,
    else parallel to the image U direction from H.
    axis \"v\" / vertical: parallel to sketch +Y when *sketch* is given,
    else parallel to the image V direction from H.

    Image calibration always passes the linked Sketch so horizontal /
    vertical mean parallel to the fixed Sketch coordinate axes; only
    bilinear image corners are optimized, not sketch placement.
    """
    d = hg._direction_xy_from_uv_line(
        line["u0"], line["v0"], line["u1"], line["v1"], H)
    if sketch is not None:
        ref_h, ref_v = _sketch_axis_directions_xy(sketch)
        ref = ref_h if axis == "u" else ref_v
    else:
        ref = hg._image_u_axis_xy(H) if axis == "u" else hg._image_v_axis_xy(H)
    if np.hypot(ref[0], ref[1]) < 1e-12 or np.hypot(d[0], d[1]) < 1e-12:
        return 1.0
    return hg._parallel_sin_xy(d, ref)


def _angle_residuals_for_constraints(
        constraints, line_by_geo, H, sketch=None):
    res = []
    for item in constraints.get("parallel", []):
        ga, gb = _constraint_line_pair(item)
        if ga not in line_by_geo or gb not in line_by_geo:
            continue
        la, lb = line_by_geo[ga], line_by_geo[gb]
        da = hg._direction_xy_from_uv_line(
            la["u0"], la["v0"], la["u1"], la["v1"], H)
        db = hg._direction_xy_from_uv_line(
            lb["u0"], lb["v0"], lb["u1"], lb["v1"], H)
        res.append(hg._parallel_sin_xy(da, db))
    for item in constraints.get("perpendicular", []):
        ga, gb = _constraint_line_pair(item)
        if ga not in line_by_geo or gb not in line_by_geo:
            continue
        la, lb = line_by_geo[ga], line_by_geo[gb]
        da = hg._direction_xy_from_uv_line(
            la["u0"], la["v0"], la["u1"], la["v1"], H)
        db = hg._direction_xy_from_uv_line(
            lb["u0"], lb["v0"], lb["u1"], lb["v1"], H)
        res.append(float(np.dot(da, db)))
    for item in constraints.get("horizontal", []):
        g = _constraint_line_index(item)
        if g not in line_by_geo:
            continue
        res.append(_axis_alignment_sin(line_by_geo[g], H, "u", sketch))
    for item in constraints.get("vertical", []):
        g = _constraint_line_index(item)
        if g not in line_by_geo:
            continue
        res.append(_axis_alignment_sin(line_by_geo[g], H, "v", sketch))
    return res


def _point_residuals_for_constraints(constraints, point_by_index, H):
    res = []
    for item in constraints.get("fixed_points", []):
        pi = _constraint_point_index(item)
        if pi not in point_by_index:
            continue
        pt = point_by_index[pi]
        pos = hg._apply_homography_uv(float(pt["u"]), float(pt["v"]), H)
        res.append(float(pos.x) - float(item["target_x_mm"]))
        res.append(float(pos.y) - float(item["target_y_mm"]))
    return res


def _weld_reference_line_endpoint_uvs(lines, img, tol_mm=None):
    """Merge endpoints within tol_mm to shared UV (topology nodes)."""
    if tol_mm is None:
        tol_mm = REF_LINE_ENDPOINT_SNAP_MM
    if not lines:
        return {}, 0

    entries = []
    for line in lines:
        u0, v0 = pa._uv_on_image(line.Start, img)
        u1, v1 = pa._uv_on_image(line.End, img)
        entries.append(((line, "start"), u0, v0, line.Start))
        entries.append(((line, "end"), u1, v1, line.End))

    n = len(entries)
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    for i in range(n):
        for j in range(i + 1, n):
            if entries[i][3].distanceToPoint(entries[j][3]) <= tol_mm:
                union(i, j)

    clusters = {}
    for i in range(n):
        clusters.setdefault(find(i), []).append(i)

    welded = {}
    welds = 0
    for members in clusters.values():
        u_mean = float(np.mean([entries[i][1] for i in members]))
        v_mean = float(np.mean([entries[i][2] for i in members]))
        if len(members) > 1:
            welds += len(members) - 1
        for i in members:
            welded[entries[i][0]] = (u_mean, v_mean)
    return welded, welds
def _reference_line_specs(lines, img, tol_mm=None):
    welded, welds = _weld_reference_line_endpoint_uvs(lines, img, tol_mm)
    specs = []
    for line in lines:
        u0, v0 = welded[(line, "start")]
        u1, v1 = welded[(line, "end")]
        specs.append({
            "label": getattr(line, "Label", "?"),
            "u0": u0, "v0": v0, "u1": u1, "v1": v1,
            "target": line.TargetLength.Value,
        })
    return specs, welds
def _length_residuals_for_specs(specs, H):
    return [
        hg._line_length_uv(sp["u0"], sp["v0"], sp["u1"], sp["v1"], H) - sp["target"]
        for sp in specs
    ]
def _corner_xy_matrix(params, z_vals):
    corners = hg._corners_from_xy_params(params, z_vals)
    return np.array([[c.x, c.y] for c in corners], dtype=float)
def _rigid_motion_stats(params, params0, z_vals):
    """Translation of centroid + rotation (Kabsch, no scale) between quad poses."""
    P0 = _corner_xy_matrix(params0, z_vals)
    P1 = _corner_xy_matrix(params, z_vals)
    g0 = P0.mean(axis=0)
    g1 = P1.mean(axis=0)
    translation_mm = float(np.linalg.norm(g1 - g0))
    A = P0 - g0
    B = P1 - g1
    if np.allclose(A, 0.0) or np.allclose(B, 0.0):
        return {"translation_mm": translation_mm, "rotation_deg": 0.0}

    H = B.T @ A
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0.0:
        Vt = Vt.copy()
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    rotation_rad = float(abs(np.arctan2(R[1, 0], R[0, 0])))
    return {
        "translation_mm": translation_mm,
        "rotation_deg": float(np.degrees(rotation_rad)),
    }
def _quad_edge_basis(params, z_vals):
    c0, cx, c1, cy = hg._corners_from_xy_params(params, z_vals)
    e_u = np.array([cx.x - c0.x, cx.y - c0.y], dtype=float)
    e_v = np.array([cy.x - c0.x, cy.y - c0.y], dtype=float)
    return np.column_stack([e_u, e_v])
_CORNER_ANGLE_LABELS = ("c0", "cx", "c1", "cy")


def _corner_edge_pairs(corners):
    """Two incident edge vectors at each quad corner (XY only)."""
    c0, cx, c1, cy = corners

    def leg(a, b):
        return np.array([b.x - a.x, b.y - a.y], dtype=float)

    return (
        (leg(c0, cx), leg(c0, cy)),
        (leg(cx, c0), leg(cx, c1)),
        (leg(c1, cx), leg(c1, cy)),
        (leg(cy, c0), leg(cy, c1)),
    )


def _edge_pair_angle_sin_cos(e_a, e_b):
    """Unit-independent sin/cos of the angle between two edge vectors."""
    na = float(np.linalg.norm(e_a))
    nb = float(np.linalg.norm(e_b))
    if na < 1e-12 or nb < 1e-12:
        return None
    cos_a = float(np.dot(e_a, e_b) / (na * nb))
    sin_a = float((e_a[0] * e_b[1] - e_a[1] * e_b[0]) / (na * nb))
    return cos_a, sin_a


def _corner_angle_energy(e_a0, e_b0, e_a1, e_b1):
    tri0 = _edge_pair_angle_sin_cos(e_a0, e_b0)
    tri1 = _edge_pair_angle_sin_cos(e_a1, e_b1)
    if tri0 is None or tri1 is None:
        return 1e12
    c0, s0 = tri0
    c1, s1 = tri1
    return (c1 - c0) ** 2 + (s1 - s0) ** 2


def _angle_preserving_energy_per_corner(params, params0, z_vals):
    """Side energy at each quad corner: zero when that interior angle is unchanged."""
    corners0 = hg._corners_from_xy_params(params0, z_vals)
    corners1 = hg._corners_from_xy_params(params, z_vals)
    pairs0 = _corner_edge_pairs(corners0)
    pairs1 = _corner_edge_pairs(corners1)
    return [
        _corner_angle_energy(a0, b0, a1, b1)
        for (a0, b0), (a1, b1) in zip(pairs0, pairs1)
    ]


def _angle_preserving_energy(params, params0, z_vals):
    """Sum of per-corner angle energies (see _angle_preserving_energy_per_corner)."""
    return float(sum(_angle_preserving_energy_per_corner(params, params0, z_vals)))


def _angle_energy_side_residuals(params, params0, z_vals):
    """Weighted sqrt(E_angle,k) side terms (corner calibration, rank < 6)."""
    w = _CALIB_ANGLE_WEIGHT
    return np.array([
        w * np.sqrt(max(E_k, 0.0))
        for E_k in _angle_preserving_energy_per_corner(params, params0, z_vals)
    ], dtype=float)


def _corner_interior_angles_deg(params, z_vals):
    angles = []
    for e_a, e_b in _corner_edge_pairs(hg._corners_from_xy_params(params, z_vals)):
        tri = _edge_pair_angle_sin_cos(e_a, e_b)
        if tri is None:
            angles.append(None)
        else:
            cos_a, sin_a = tri
            angles.append(float(np.degrees(np.arctan2(sin_a, cos_a))))
    return angles


def _uv_basis_angle_deg(params, z_vals):
    """Interior angle at c0 (backward-compatible alias)."""
    angles = _corner_interior_angles_deg(params, z_vals)
    return angles[0] if angles else None


def _angle_energy_report(params, params0, z_vals):
    corner_energies = _angle_preserving_energy_per_corner(params, params0, z_vals)
    angles = _corner_interior_angles_deg(params, z_vals)
    angles0 = _corner_interior_angles_deg(params0, z_vals)
    E = float(sum(corner_energies))
    return {
        "angle_preserving_energy": E,
        "distortion_energy": E,
        "corner_angle_energies": corner_energies,
        "corner_angles_deg": angles,
        "corner_angles_deg0": angles0,
        "uv_corner_angle_deg": angles[0] if angles else None,
        "uv_corner_angle_deg0": angles0[0] if angles0 else None,
    }


def _distortion_energy(params, params0, z_vals):
    """Backward-compatible alias for angle-preserving side energy."""
    return _angle_preserving_energy(params, params0, z_vals)


def _primary_calibration_residuals(
        params, specs, z_vals, constraints=None, line_by_geo=None,
        sketch=None, point_by_index=None):
    """Length, angle, and fixed-point constraint values."""
    H = hg._homography_from_xy_params(params, z_vals)
    parts = list(_length_residuals_for_specs(specs, H))
    if constraints is not None and line_by_geo is not None:
        parts.extend(_angle_residuals_for_constraints(
            constraints, line_by_geo, H, sketch=sketch))
    if constraints is not None and point_by_index is not None:
        parts.extend(_point_residuals_for_constraints(
            constraints, point_by_index, H))
    if not parts:
        return np.zeros(0, dtype=float)
    return np.asarray(parts, dtype=float)


def _numerical_jacobian(func, x0, eps=_JACOBIAN_FD_EPS):
    x0 = np.asarray(x0, dtype=float)
    r0 = func(x0)
    n = x0.size
    J = np.zeros((r0.size, n), dtype=float)
    for j in range(n):
        x = x0.copy()
        x[j] += eps
        J[:, j] = (func(x) - r0) / eps
    return J


def _matrix_rank(J, rtol=_JACOBIAN_RANK_RTOL, atol=_JACOBIAN_RANK_ATOL):
    if J.size == 0:
        return 0
    s = np.linalg.svd(J, compute_uv=False)
    if s.size == 0 or s[0] == 0.0:
        return 0
    tol = max(atol, rtol * float(s[0]))
    return int(np.sum(s > tol))


def _primary_constraint_rank(
        params0, specs, z_vals, constraints=None, line_by_geo=None,
        sketch=None, point_by_index=None):
    """Rank of primary constraint Jacobian at *params0*.

    Returns (rank, n_primary, include_angle_energy).
    E_angle side terms apply only when rank < effective DOF (6).
    """

    def func(params):
        return _primary_calibration_residuals(
            params, specs, z_vals,
            constraints=constraints, line_by_geo=line_by_geo,
            sketch=sketch, point_by_index=point_by_index)

    n_primary = int(func(params0).size)
    if n_primary == 0:
        return 0, 0, True
    J = _numerical_jacobian(func, params0)
    rank = _matrix_rank(J)
    include_angle = rank < _CORNER_EFFECTIVE_DOF
    return rank, n_primary, include_angle


def _determinacy_label(rank, n_primary, effective_dof=_CORNER_EFFECTIVE_DOF):
    """Classify primary constraint system at the start point."""
    if rank < effective_dof:
        return "unterbestimmt"
    if n_primary > effective_dof:
        return "überbestimmt"
    return "bestimmt"


def _count_angle_constraints(constraints, line_by_geo):
    if not constraints or not line_by_geo:
        return 0
    n = 0
    for item in constraints.get("parallel", []):
        ga, gb = _constraint_line_pair(item)
        if ga in line_by_geo and gb in line_by_geo:
            n += 1
    for item in constraints.get("perpendicular", []):
        ga, gb = _constraint_line_pair(item)
        if ga in line_by_geo and gb in line_by_geo:
            n += 1
    for key in ("horizontal", "vertical"):
        for item in constraints.get(key, []):
            if _constraint_line_index(item) in line_by_geo:
                n += 1
    return n


def _solver_diagnostics_meta(
        rank, n_primary, include_angle_energy, mode,
        n_lengths=None, n_angles=None, include_centroid=True):
    det = _determinacy_label(rank, n_primary)
    centroid_in_residuals = mode == "corners" and include_centroid
    return {
        "mode": mode,
        "constraint_rank": rank,
        "primary_constraint_count": n_primary,
        "corner_param_dof": _CORNER_PARAM_DOF,
        "effective_dof": _CORNER_EFFECTIVE_DOF,
        "include_side_terms": include_angle_energy,
        "include_translation_side": centroid_in_residuals,
        "include_distortion_energy": include_angle_energy,
        "determinacy": det,
        "n_length_constraints": n_lengths,
        "n_angle_constraints": n_angles,
    }


_LEAST_SQUARES_STATUS_LABELS = {
    -1: "ungültige Eingabe",
    0: "max. Funktionsauswertungen (max_nfev)",
    1: "gtol erreicht",
    2: "ftol erreicht",
    3: "xtol erreicht",
    4: "ftol und xtol erreicht",
}


def _least_squares_status_text(status):
    try:
        code = int(status)
    except (TypeError, ValueError):
        return str(status)
    label = _LEAST_SQUARES_STATUS_LABELS.get(code)
    if label:
        return "{} ({})".format(label, code)
    return "Status {}".format(code)


def _print_solver_diagnostics(meta=None, opt_info=None, constraints=None,
                              line_by_geo=None):
    """Debug block: rank, determinacy, side terms, optimizer metrics."""
    if meta is None:
        return
    App.Console.PrintMessage("\n=== Solver-Diagnose ===\n")
    mode = meta.get("mode", "?")
    mode_labels = {
        "corners": "Eckpunkt-Optimierung (least_squares)",
        "uniform_scale": "Einheitliche Skalierung (1D)",
        "uv_scale": "UV-Skalierung sx/sy (2D)",
    }
    App.Console.PrintMessage(
        "  Modus: {}\n".format(mode_labels.get(mode, mode)))

    n_len = meta.get("n_length_constraints")
    n_ang = meta.get("n_angle_constraints")
    if n_len is None and constraints is not None:
        n_len = len(constraints.get("lengths", []))
    if n_ang is None and constraints is not None and line_by_geo is not None:
        n_ang = _count_angle_constraints(constraints, line_by_geo)
    if n_len is not None or n_ang is not None:
        App.Console.PrintMessage(
            "  Primäre Bedingungen: {} ({} Längen, {} Winkel)\n".format(
                meta.get("primary_constraint_count", "?"),
                n_len if n_len is not None else "?",
                n_ang if n_ang is not None else "?"))

    rank = meta.get("constraint_rank")
    n_primary = meta.get("primary_constraint_count")
    eff = meta.get("effective_dof", _CORNER_EFFECTIVE_DOF)
    param_dof = meta.get("corner_param_dof", _CORNER_PARAM_DOF)
    if rank is not None:
        App.Console.PrintMessage(
            "  Jacobian-Rang: {} / {} Gleichungen "
            "({} Eckparameter, {} eff. DOF)\n".format(
                rank, n_primary, param_dof, eff))
    det = meta.get("determinacy")
    if det:
        App.Console.PrintMessage("  Bestimmtheit: {}\n".format(det))
    if "include_translation_side" in meta or "include_side_terms" in meta:
        e_side = meta.get("include_distortion_energy", meta.get("include_side_terms"))
        trans = meta.get("include_translation_side")
        App.Console.PrintMessage(
            "  Winkelerhaltung E_angle in Optimierung: {}\n".format(
                "ja" if e_side else "nein"))
        App.Console.PrintMessage(
            "  Schwerpunkt-Nebenbedingung in Optimierung: {}\n".format(
                "ja" if trans else "nein"))

    if meta.get("stop_phase"):
        App.Console.PrintMessage(
            "  Abbruch nach Phase: {}\n".format(meta["stop_phase"]))
    if mode == "uniform_scale" and meta.get("scale_factor") is not None:
        App.Console.PrintMessage(
            "  Skalenfaktor: {:.6f}\n".format(meta["scale_factor"]))
    elif mode == "uniform_scale" and opt_info and opt_info.get("scale_factor"):
        App.Console.PrintMessage(
            "  Skalenfaktor: {:.6f}\n".format(opt_info["scale_factor"]))
    elif mode == "uv_scale" and meta.get("scale_sx") is not None:
        App.Console.PrintMessage(
            "  UV-Skalierung: sx = {:.6f}, sy = {:.6f}\n".format(
                meta.get("scale_sx", 0.0), meta.get("scale_sy", 0.0)))
    if meta.get("uniform_scale_factor") is not None and mode != "uniform_scale":
        App.Console.PrintMessage(
            "  Phase 1 (uniform): s = {:.6f}\n".format(
                meta["uniform_scale_factor"]))
    if meta.get("uv_scale_warm_start"):
        App.Console.PrintMessage(
            "  UV-Warmstart: sx = {:.6f}, sy = {:.6f} "
            "(success = {}, nfev = {})\n".format(
                meta.get("scale_sx", 0.0),
                meta.get("scale_sy", 0.0),
                meta.get("uv_scale_success"),
                meta.get("uv_scale_nfev")))

    if opt_info is not None:
        App.Console.PrintMessage("  Optimierung:\n")
        App.Console.PrintMessage(
            "    success = {}, cost = {:.6e}\n".format(
                opt_info.get("success", True),
                opt_info.get("cost", 0.0)))
        if "nfev" in opt_info:
            App.Console.PrintMessage(
                "    nfev = {}\n".format(opt_info["nfev"]))
        if "status" in opt_info:
            App.Console.PrintMessage(
                "    {}\n".format(
                    _least_squares_status_text(opt_info["status"])))
        if opt_info.get("message"):
            App.Console.PrintMessage(
                "    message: {}\n".format(opt_info["message"]))
        if opt_info.get("refined"):
            App.Console.PrintMessage("    Verfeinerung: ja\n")
        elif mode == "corners":
            App.Console.PrintMessage("    Verfeinerung: nein\n")

    if meta.get("angle_preserving_energy") is not None:
        App.Console.PrintMessage(
            "  Winkelerhaltung E_angle gesamt = {:.6e} "
            "(0 = alle Eckwinkel unverändert)\n".format(
                meta["angle_preserving_energy"]))
        corner_energies = meta.get("corner_angle_energies")
        if corner_energies:
            for label, E_k in zip(_CORNER_ANGLE_LABELS, corner_energies):
                App.Console.PrintMessage(
                    "    E_angle {} = {:.6e}\n".format(label, E_k))
        angles0 = meta.get("corner_angles_deg0")
        angles1 = meta.get("corner_angles_deg")
        if angles0 and angles1:
            for label, a0, a1 in zip(
                    _CORNER_ANGLE_LABELS, angles0, angles1):
                if a0 is not None and a1 is not None:
                    App.Console.PrintMessage(
                        "    Winkel {}: {:.3f}° → {:.3f}° "
                        "(Δ {:+.3f}°)\n".format(label, a0, a1, a1 - a0))
        elif meta.get("uv_corner_angle_deg0") is not None:
            a0 = meta.get("uv_corner_angle_deg0")
            a1 = meta.get("uv_corner_angle_deg")
            App.Console.PrintMessage(
                "  U/V-Winkel an c0: {:.3f}° → {:.3f}° "
                "(Δ {:+.3f}°)\n".format(a0, a1, a1 - a0))
    elif meta.get("distortion_energy") is not None:
        App.Console.PrintMessage(
            "  Winkelerhaltung E_angle = {:.6e}\n".format(
                meta["distortion_energy"]))
    rigid = meta.get("rigid_motion")
    if rigid:
        App.Console.PrintMessage(
            "  Starre Bewegung: Δ = {:.4f} mm, θ = {:.4f}°\n".format(
                rigid.get("translation_mm", 0.0),
                rigid.get("rotation_deg", 0.0)))
    move = meta.get("corner_move")
    if move:
        App.Console.PrintMessage(
            "  Eck-Verschiebung: max = {:.4f} mm, rms = {:.4f} mm\n".format(
                move.get("max_mm", 0.0), move.get("rms_mm", 0.0)))
    if meta.get("exact") is not None:
        App.Console.PrintMessage(
            "  Längen exakt (< {:.3f} mm): {}\n".format(
                _CALIB_LENGTH_TOLERANCE_MM, meta.get("exact")))


def _calibration_residuals(
        params, specs, params0, z_vals, constraints=None, line_by_geo=None,
        sketch=None, point_by_index=None, include_angle_energy=True,
        include_centroid=True):
    """Build full residual vector for corner calibration (phase 3).

    *include_angle_energy* (rank < 6): append w·√E_angle.
    Centroid translation Δt/τ_t is appended when *include_centroid* (default on).
    """
    H = hg._homography_from_xy_params(params, z_vals)
    length_res = np.asarray(_length_residuals_for_specs(specs, H), dtype=float)
    length_part = length_res / _CALIB_LENGTH_TOLERANCE_MM
    rigid = _rigid_motion_stats(params, params0, z_vals)
    rigid_part = np.array([
        rigid["translation_mm"] / _CALIB_RIGID_TRANSLATION_TOLERANCE_MM,
    ], dtype=float)
    parts = [length_part]
    if constraints is not None and line_by_geo is not None:
        angle_res = _angle_residuals_for_constraints(
            constraints, line_by_geo, H, sketch=sketch)
        if angle_res:
            parts.append(
                np.asarray(angle_res, dtype=float)
                / _CALIB_ANGLE_TOLERANCE_RAD
                * _CALIB_ANGLE_WEIGHT)
    if constraints is not None and point_by_index is not None:
        point_res = _point_residuals_for_constraints(
            constraints, point_by_index, H)
        if point_res:
            parts.append(
                np.asarray(point_res, dtype=float) / _CALIB_POINT_TOLERANCE_MM)
    if include_angle_energy:
        parts.append(_angle_energy_side_residuals(params, params0, z_vals))
    if include_centroid:
        parts.append(rigid_part)
    return np.concatenate(parts)


def _deviation_deg_from_sin(sin_val):
    return abs(float(np.degrees(np.arcsin(np.clip(sin_val, -1.0, 1.0)))))


def _deviation_deg_from_perpendicular_dot(dot_val):
    return abs(
        90.0 - np.degrees(np.arccos(np.clip(abs(dot_val), 0.0, 1.0))))


def _print_axis_constraint_report(constraints, line_by_geo, H, sketch=None):
    for label, key, axis in (
            ("Horizontal", "horizontal", "u"),
            ("Senkrecht", "vertical", "v")):
        for item in constraints.get(key, []):
            g = _constraint_line_index(item)
            if g not in line_by_geo:
                App.Console.PrintWarning(
                    "  {}: Kante L{} nicht im Sketch.\n".format(label, g))
                continue
            sin_a = _axis_alignment_sin(line_by_geo[g], H, axis, sketch)
            deg = _deviation_deg_from_sin(sin_a)
            App.Console.PrintMessage(
                "  {} L{}: Abweichung {:.3f}°\n".format(label, g, deg))


def _print_calibration_constraint_report(
        constraints, line_by_geo, H, length_specs, sketch=None,
        meta=None, opt_info=None):
    """Report residual error for every active calibration constraint."""
    _print_solver_diagnostics(
        meta=meta, opt_info=opt_info,
        constraints=constraints, line_by_geo=line_by_geo)
    App.Console.PrintMessage("\n=== Bedingungen nach Kalibrierung ===\n")

    max_len_delta = 0.0
    if length_specs:
        App.Console.PrintMessage("Soll-Längen:\n")
        for spec in length_specs:
            actual = _predicted_line_length_spec(spec, H)
            target = float(spec["target"])
            delta = actual - target
            max_len_delta = max(max_len_delta, abs(delta))
            App.Console.PrintMessage(
                "  {}: Ist {:.4f} mm, Soll {:.4f} mm, Δ {:+.4f} mm\n".format(
                    spec.get("label", "?"), actual, target, delta))
    else:
        App.Console.PrintMessage("Soll-Längen: (keine)\n")

    max_angle_delta = 0.0

    def report_axis(label, key, axis):
        nonlocal max_angle_delta
        items = constraints.get(key, [])
        if not items:
            return
        App.Console.PrintMessage("{}:\n".format(label))
        for item in items:
            g = _constraint_line_index(item)
            if g not in line_by_geo:
                App.Console.PrintWarning(
                    "  L{}: nicht im Sketch.\n".format(g))
                continue
            sin_a = _axis_alignment_sin(line_by_geo[g], H, axis, sketch)
            deg = _deviation_deg_from_sin(sin_a)
            max_angle_delta = max(max_angle_delta, deg)
            App.Console.PrintMessage(
                "  L{}: Abweichung {:.3f}°\n".format(g, deg))

    report_axis("Horizontal (parallel Sketch +X)", "horizontal", "u")
    report_axis("Senkrecht (parallel Sketch +Y)", "vertical", "v")

    parallel_items = constraints.get("parallel", [])
    if parallel_items:
        App.Console.PrintMessage("Parallel:\n")
        for item in parallel_items:
            ga, gb = _constraint_line_pair(item)
            if ga not in line_by_geo or gb not in line_by_geo:
                App.Console.PrintWarning(
                    "  L{} ∥ L{}: Kante fehlt im Sketch.\n".format(ga, gb))
                continue
            la, lb = line_by_geo[ga], line_by_geo[gb]
            da = hg._direction_xy_from_uv_line(
                la["u0"], la["v0"], la["u1"], la["v1"], H)
            db = hg._direction_xy_from_uv_line(
                lb["u0"], lb["v0"], lb["u1"], lb["v1"], H)
            deg = _deviation_deg_from_sin(hg._parallel_sin_xy(da, db))
            max_angle_delta = max(max_angle_delta, deg)
            App.Console.PrintMessage(
                "  L{} ∥ L{}: Abweichung {:.3f}°\n".format(ga, gb, deg))

    perp_items = constraints.get("perpendicular", [])
    if perp_items:
        App.Console.PrintMessage("Rechtwinklig:\n")
        for item in perp_items:
            ga, gb = _constraint_line_pair(item)
            if ga not in line_by_geo or gb not in line_by_geo:
                App.Console.PrintWarning(
                    "  L{} ⊥ L{}: Kante fehlt im Sketch.\n".format(ga, gb))
                continue
            la, lb = line_by_geo[ga], line_by_geo[gb]
            da = hg._direction_xy_from_uv_line(
                la["u0"], la["v0"], la["u1"], la["v1"], H)
            db = hg._direction_xy_from_uv_line(
                lb["u0"], lb["v0"], lb["u1"], lb["v1"], H)
            dot = float(np.dot(da, db))
            deg = _deviation_deg_from_perpendicular_dot(dot)
            max_angle_delta = max(max_angle_delta, deg)
            App.Console.PrintMessage(
                "  L{} ⊥ L{}: Abweichung {:.3f}° von 90°\n".format(
                    ga, gb, deg))

    fixed_items = constraints.get("fixed_points", [])
    point_by_index = (meta or {}).get("point_by_index") or {}
    max_point_delta = 0.0
    if fixed_items:
        App.Console.PrintMessage("Fixpunkte:\n")
        for item in fixed_items:
            pi = _constraint_point_index(item)
            if pi not in point_by_index:
                App.Console.PrintWarning(
                    "  V{}: nicht im Sketch.\n".format(pi))
                continue
            pt = point_by_index[pi]
            pt_label = pt.get("label", "V{}".format(pi))
            pos = hg._apply_homography_uv(float(pt["u"]), float(pt["v"]), H)
            tx = float(item["target_x_mm"])
            ty = float(item["target_y_mm"])
            dx = float(pos.x) - tx
            dy = float(pos.y) - ty
            max_point_delta = max(max_point_delta, abs(dx), abs(dy))
            App.Console.PrintMessage(
                "  {}: Ist ({:.4f}, {:.4f}), Soll ({:.4f}, {:.4f}), "
                "Δ ({:+.4f}, {:+.4f}) mm\n".format(
                    pt_label, pos.x, pos.y, tx, ty, dx, dy))

    App.Console.PrintMessage("\nZusammenfassung:\n")
    if length_specs:
        App.Console.PrintMessage(
            "  Längen: max |Δ| = {:.4f} mm "
            "(Toleranz {:.3f} mm)\n".format(
                max_len_delta, _CALIB_LENGTH_TOLERANCE_MM))
    if max_angle_delta > 0.0:
        App.Console.PrintMessage(
            "  Winkel: max Abweichung = {:.3f}° "
            "(Toleranz {:.1f}°)\n".format(
                max_angle_delta, np.degrees(np.arcsin(_CALIB_ANGLE_TOLERANCE_RAD))))
    if max_point_delta > 0.0:
        App.Console.PrintMessage(
            "  Fixpunkte: max |Δ| = {:.4f} mm "
            "(Toleranz {:.3f} mm)\n".format(
                max_point_delta, _CALIB_POINT_TOLERANCE_MM))
    if meta is not None:
        if "exact" in meta:
            App.Console.PrintMessage(
                "  Längen exakt (< {:.3f} mm): {}\n".format(
                    _CALIB_LENGTH_TOLERANCE_MM, meta.get("exact", False)))
        res = opt_info.get("residuals") if opt_info else None
        if res is not None and length_specs:
            parts = []
            for spec, r in zip(length_specs, res):
                parts.append("{}={:+.4f}".format(
                    spec.get("label", "?"), float(r)))
            if parts:
                App.Console.PrintMessage(
                    "  Restfehler Längen [mm]: {}\n".format(", ".join(parts)))


def _can_use_uniform_scale_solver(specs, constraints):
    """Single length, no angle constraints → uniform scale about quad centroid."""
    if len(specs) != 1:
        return False
    return not _has_angle_constraints(constraints)


def _quad_centroid_xy(params0):
    """Centroid of the four quad corners in XY."""
    p = np.asarray(params0, dtype=float)
    return np.array([
        0.25 * (p[0] + p[2] + p[4] + p[6]),
        0.25 * (p[1] + p[3] + p[5] + p[7]),
    ], dtype=float)


_UV_CORNER_UV = (
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
)


def _uniform_scale_params(params0, scale):
    """Scale all quad corners about centroid; centroid stays fixed."""
    params = np.asarray(params0, dtype=float)
    g = _quad_centroid_xy(params)
    s = float(scale)
    out = []
    for i in range(0, len(params), 2):
        out.append(g[0] + s * (params0[i] - g[0]))
        out.append(g[1] + s * (params0[i + 1] - g[1]))
    return np.array(out, dtype=float)


def _max_length_error_mm(specs, H):
    res = _length_residuals_for_specs(specs, H)
    return max((abs(r) for r in res), default=0.0)


def _translate_params_xy(params, tx, ty):
    out = np.asarray(params, dtype=float).copy()
    out[0::2] += float(tx)
    out[1::2] += float(ty)
    return out


def _uniform_scale_phase_params(params0, scale, tx=0.0, ty=0.0):
    return _translate_params_xy(_uniform_scale_params(params0, scale), tx, ty)


def _uv_scale_phase_params(params0, sx, sy, tx=0.0, ty=0.0):
    return _translate_params_xy(_uv_scale_params(params0, sx, sy), tx, ty)


def _max_fixed_point_error_mm(constraints, point_by_index, H):
    res = _point_residuals_for_constraints(
        constraints or {}, point_by_index or {}, H)
    if not res:
        return 0.0
    return max(abs(float(r)) for r in res)


def _scale_phase_converged(
        specs, params, params0, z_vals, constraints=None, point_by_index=None):
    """True when length targets are met and corner angles are unchanged."""
    H = hg._homography_from_xy_params(params, z_vals)
    if _max_length_error_mm(specs, H) >= _CALIB_LENGTH_TOLERANCE_MM:
        return False
    if _angle_preserving_energy(params, params0, z_vals) >= 1e-6:
        return False
    if _has_fixed_point_constraints(constraints, point_by_index):
        if (_max_fixed_point_error_mm(constraints, point_by_index, H)
                >= _CALIB_POINT_TOLERANCE_MM):
            return False
    return True


def _uniform_scale_phase_residuals(
        x, specs, params_in, z_vals, constraints=None, point_by_index=None):
    s = float(x[0])
    tx = float(x[1]) if len(x) > 1 else 0.0
    ty = float(x[2]) if len(x) > 2 else 0.0
    params = _uniform_scale_phase_params(params_in, s, tx, ty)
    H = hg._homography_from_xy_params(params, z_vals)
    parts = [
        r / _CALIB_LENGTH_TOLERANCE_MM
        for r in _length_residuals_for_specs(specs, H)]
    if _has_fixed_point_constraints(constraints, point_by_index):
        pt_res = _point_residuals_for_constraints(
            constraints, point_by_index, H)
        parts.extend([r / _CALIB_POINT_TOLERANCE_MM for r in pt_res])
    return np.asarray(parts, dtype=float)


def _uniform_scale_length_residuals(scale, specs, params_in, z_vals):
    return _uniform_scale_phase_residuals(
        scale, specs, params_in, z_vals)


def _apply_uniform_scale_phase(
        specs, params_in, z_vals, constraints=None, point_by_index=None):
    """Phase 1: uniform scale (analytic for one length, else least_squares on s)."""
    params_in = np.asarray(params_in, dtype=float)
    use_fixed = _has_fixed_point_constraints(constraints, point_by_index)
    if len(specs) == 1 and not use_fixed:
        corners = hg._corners_from_xy_params(params_in, z_vals)
        H0 = hg._homography_from_corners(*corners)
        spec = specs[0]
        current = hg._line_length_uv(
            spec["u0"], spec["v0"], spec["u1"], spec["v1"], H0)
        if current < 1e-12:
            raise ValueError("Degenerate line for uniform scale")
        scale = float(spec["target"]) / current
        params = _uniform_scale_phase_params(params_in, scale)
        info = {
            "scale_factor": scale,
            "translation_x_mm": 0.0,
            "translation_y_mm": 0.0,
            "analytic": True,
            "nfev": 0,
            "success": True,
            "cost": 0.0,
        }
        return params, info

    x0 = [1.0]
    if use_fixed:
        x0.extend([0.0, 0.0])
    result = least_squares(
        lambda x: _uniform_scale_phase_residuals(
            x, specs, params_in, z_vals, constraints, point_by_index),
        np.array(x0, dtype=float),
        ftol=_CALIB_SOLVER_FTOL,
        xtol=_CALIB_SOLVER_XTOL,
        gtol=_CALIB_SOLVER_GTOL,
        max_nfev=_CALIB_SOLVER_MAX_NFEV)
    s = float(result.x[0])
    tx = float(result.x[1]) if use_fixed else 0.0
    ty = float(result.x[2]) if use_fixed else 0.0
    params = _uniform_scale_phase_params(params_in, s, tx, ty)
    info = {
        "scale_factor": s,
        "translation_x_mm": tx,
        "translation_y_mm": ty,
        "analytic": False,
        "nfev": int(result.nfev),
        "success": bool(result.success),
        "cost": float(result.cost),
    }
    return params, info


def _make_uniform_scale_result(
        params, params0, z_vals, specs, scale_info,
        constraints=None, line_by_geo=None, point_by_index=None, sketch=None):
    corners = hg._corners_from_xy_params(params, z_vals)
    H = hg._homography_from_corners(*corners)
    length_res = _length_residuals_for_specs(specs, H)
    max_len_err = _max_length_error_mm(specs, H)
    rigid = _rigid_motion_stats(params, params0, z_vals)
    move = _corner_movement_stats(params, params0)
    angle_meta = _angle_energy_report(params, params0, z_vals)
    include_centroid = not _has_fixed_point_constraints(
        constraints, point_by_index)
    rank, n_primary, include_trans = _primary_constraint_rank(
        params0, specs, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
        point_by_index=point_by_index)
    n_angles = _count_angle_constraints(constraints, line_by_geo)
    opt_info = {
        "cost": scale_info.get("cost", 0.0),
        "success": scale_info.get("success", True),
        "residuals": length_res,
        "rigid_motion": rigid,
        "corner_move": move,
        "scale_factor": scale_info["scale_factor"],
        "stop_phase": "uniform_scale",
        "nfev": scale_info.get("nfev", 0),
        **angle_meta,
    }
    meta = {
        "mode": "uniform_scale",
        "stop_phase": "uniform_scale",
        "exact": max_len_err < _CALIB_LENGTH_TOLERANCE_MM,
        "rigid_motion": rigid,
        "corners": corners,
        "corner_move": move,
        "scale_factor": scale_info["scale_factor"],
        "point_by_index": point_by_index or {},
        **angle_meta,
        **_solver_diagnostics_meta(
            rank, n_primary, include_trans, "uniform_scale",
            n_lengths=len(specs), n_angles=n_angles,
            include_centroid=include_centroid),
    }
    return corners, H, opt_info, meta


def _make_uv_scale_result(
        params, params0, z_vals, specs, uniform_info, uv_info,
        constraints=None, line_by_geo=None, point_by_index=None, sketch=None):
    corners = hg._corners_from_xy_params(params, z_vals)
    H = hg._homography_from_corners(*corners)
    length_res = _length_residuals_for_specs(specs, H)
    max_len_err = _max_length_error_mm(specs, H)
    rigid = _rigid_motion_stats(params, params0, z_vals)
    move = _corner_movement_stats(params, params0)
    angle_meta = _angle_energy_report(params, params0, z_vals)
    include_centroid = not _has_fixed_point_constraints(
        constraints, point_by_index)
    rank, n_primary, include_trans = _primary_constraint_rank(
        params0, specs, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
        point_by_index=point_by_index)
    n_angles = _count_angle_constraints(constraints, line_by_geo)
    opt_info = {
        "cost": uv_info.get("cost", 0.0),
        "success": uv_info.get("success", True),
        "residuals": length_res,
        "rigid_motion": rigid,
        "corner_move": move,
        "stop_phase": "uv_scale",
        "scale_sx": uv_info["scale_sx"],
        "scale_sy": uv_info["scale_sy"],
        "uv_scale_nfev": uv_info.get("nfev", 0),
        "nfev": uv_info.get("nfev", 0),
        **angle_meta,
    }
    if uniform_info is not None:
        opt_info["uniform_scale_factor"] = uniform_info.get("scale_factor")
    meta = {
        "mode": "uv_scale",
        "stop_phase": "uv_scale",
        "exact": max_len_err < _CALIB_LENGTH_TOLERANCE_MM,
        "rigid_motion": rigid,
        "corners": corners,
        "corner_move": move,
        "scale_sx": uv_info["scale_sx"],
        "scale_sy": uv_info["scale_sy"],
        "uv_scale_success": uv_info.get("success"),
        "uv_scale_nfev": uv_info.get("nfev", 0),
        "point_by_index": point_by_index or {},
        **angle_meta,
        **_solver_diagnostics_meta(
            rank, n_primary, include_trans, "uv_scale",
            n_lengths=len(specs), n_angles=n_angles,
            include_centroid=include_centroid),
    }
    if uniform_info is not None:
        meta["uniform_scale_factor"] = uniform_info.get("scale_factor")
    return corners, H, opt_info, meta


def _solve_uniform_scale_calibration(specs, params0, z_vals):
    """Analytic uniform scale from one target length (similarity about centroid)."""
    params, scale_info = _apply_uniform_scale_phase(specs, params0, z_vals)
    return _make_uniform_scale_result(
        params, params0, z_vals, specs, scale_info)


def _can_use_uv_scale_warm_start(specs, constraints):
    """Two length targets, no angle constraints → sx/sy warm start."""
    if len(specs) != 2:
        return False
    return not _has_angle_constraints(constraints)


def _uv_scale_params(params0, sx, sy):
    """Scale U/V about quad centroid (u,v pivot 0.5); preserves angles and centroid."""
    p = np.asarray(params0, dtype=float)
    c0 = p[0:2]
    e_u = p[2:4] - c0
    e_v = p[6:8] - c0
    g = _quad_centroid_xy(p)
    sx_f, sy_f = float(sx), float(sy)
    out = []
    for u, v in _UV_CORNER_UV:
        corner = g + sx_f * (u - 0.5) * e_u + sy_f * (v - 0.5) * e_v
        out.extend(corner)
    return np.array(out, dtype=float)


def _uv_scale_phase_residuals(
        x, specs, params0, z_vals, constraints=None, point_by_index=None):
    sx = float(x[0])
    sy = float(x[1])
    tx = float(x[2]) if len(x) > 2 else 0.0
    ty = float(x[3]) if len(x) > 3 else 0.0
    params = _uv_scale_phase_params(params0, sx, sy, tx, ty)
    H = hg._homography_from_xy_params(params, z_vals)
    parts = [
        r / _CALIB_LENGTH_TOLERANCE_MM
        for r in _length_residuals_for_specs(specs, H)]
    if _has_fixed_point_constraints(constraints, point_by_index):
        pt_res = _point_residuals_for_constraints(
            constraints, point_by_index, H)
        parts.extend([r / _CALIB_POINT_TOLERANCE_MM for r in pt_res])
    return np.asarray(parts, dtype=float)


def _uv_scale_length_residuals(scales, specs, params0, z_vals):
    return _uv_scale_phase_residuals(scales, specs, params0, z_vals)


def _apply_uv_scale_phase(
        specs, params_in, z_vals, constraints=None, point_by_index=None):
    """Phase 2: independent U/V scale (2 DOF) about current quad pose."""
    use_fixed = _has_fixed_point_constraints(constraints, point_by_index)
    x0 = [1.0, 1.0]
    if use_fixed:
        x0.extend([0.0, 0.0])
    result = least_squares(
        lambda x: _uv_scale_phase_residuals(
            x, specs, params_in, z_vals, constraints, point_by_index),
        np.array(x0, dtype=float),
        ftol=_CALIB_SOLVER_FTOL,
        xtol=_CALIB_SOLVER_XTOL,
        gtol=_CALIB_SOLVER_GTOL,
        max_nfev=_CALIB_SOLVER_MAX_NFEV)
    sx, sy = float(result.x[0]), float(result.x[1])
    tx = float(result.x[2]) if use_fixed else 0.0
    ty = float(result.x[3]) if use_fixed else 0.0
    params = _uv_scale_phase_params(params_in, sx, sy, tx, ty)
    info = {
        "scale_sx": sx,
        "scale_sy": sy,
        "translation_x_mm": tx,
        "translation_y_mm": ty,
        "success": bool(result.success),
        "nfev": int(result.nfev),
        "cost": float(result.cost),
    }
    return params, info


def _solve_uv_scale_phase(specs, params0, z_vals):
    """Backward-compatible alias for phase-2 UV scale."""
    params, info = _apply_uv_scale_phase(specs, params0, z_vals)
    return params, info["scale_sx"], info["scale_sy"], info


def _solve_corner_calibration(
        specs, params0, z_vals, constraints=None, line_by_geo=None,
        sketch=None, point_by_index=None):
    if least_squares is None:
        raise RuntimeError("scipy is required for corner calibration")

    params_curr = np.asarray(params0, dtype=float).copy()
    allow_scale_exit = not _has_angle_constraints(constraints)
    uniform_info = None
    uv_info = None
    include_centroid = not _has_fixed_point_constraints(
        constraints, point_by_index)

    # Phase 1 — uniform (1D) scale (always)
    params_curr, uniform_info = _apply_uniform_scale_phase(
        specs, params_curr, z_vals,
        constraints=constraints, point_by_index=point_by_index)
    if allow_scale_exit and _scale_phase_converged(
            specs, params_curr, params0, z_vals,
            constraints=constraints, point_by_index=point_by_index):
        return _make_uniform_scale_result(
            params_curr, params0, z_vals, specs, uniform_info,
            constraints=constraints, line_by_geo=line_by_geo,
            point_by_index=point_by_index, sketch=sketch)

    # Phase 2 — independent U/V (2D) scale (always after phase 1)
    params_curr, uv_info = _apply_uv_scale_phase(
        specs, params_curr, z_vals,
        constraints=constraints, point_by_index=point_by_index)
    if allow_scale_exit and _scale_phase_converged(
            specs, params_curr, params0, z_vals,
            constraints=constraints, point_by_index=point_by_index):
        return _make_uv_scale_result(
            params_curr, params0, z_vals, specs,
            uniform_info, uv_info,
            constraints=constraints, line_by_geo=line_by_geo,
            point_by_index=point_by_index, sketch=sketch)

    # Phase 3 — full corner optimization
    params_start = params_curr
    rank, n_primary, include_angle = _primary_constraint_rank(
        params0, specs, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
        point_by_index=point_by_index)
    n_angles = _count_angle_constraints(constraints, line_by_geo)

    residual_fn = lambda params: _calibration_residuals(
        params, specs, params0, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
        point_by_index=point_by_index,
        include_angle_energy=include_angle,
        include_centroid=include_centroid)

    result = least_squares(
        residual_fn,
        params_start,
        ftol=_CALIB_SOLVER_FTOL,
        xtol=_CALIB_SOLVER_XTOL,
        gtol=_CALIB_SOLVER_GTOL,
        max_nfev=_CALIB_SOLVER_MAX_NFEV)

    params = result.x
    corners = hg._corners_from_xy_params(params, z_vals)
    H = hg._homography_from_corners(*corners)
    length_res = _length_residuals_for_specs(specs, H)
    max_len_err = _max_length_error_mm(specs, H)
    refined = False

    if max_len_err >= _CALIB_LENGTH_TOLERANCE_MM:
        refine = least_squares(
            residual_fn,
            params,
            ftol=_CALIB_SOLVER_REFINE_FTOL,
            xtol=_CALIB_SOLVER_REFINE_FTOL,
            gtol=_CALIB_SOLVER_REFINE_FTOL,
            max_nfev=_CALIB_SOLVER_REFINE_MAX_NFEV)
        params = refine.x
        result = refine
        refined = True
        corners = hg._corners_from_xy_params(params, z_vals)
        H = hg._homography_from_corners(*corners)
        length_res = _length_residuals_for_specs(specs, H)
        max_len_err = _max_length_error_mm(specs, H)

    rigid = _rigid_motion_stats(params, params0, z_vals)
    move = _corner_movement_stats(params, params0)
    angle_meta = _angle_energy_report(params, params0, z_vals)
    opt_info = {
        "cost": result.cost,
        "success": result.success,
        "residuals": length_res,
        "rigid_motion": rigid,
        "corner_move": move,
        "nfev": int(result.nfev),
        "status": int(result.status),
        "message": str(result.message),
        "refined": refined,
        "stop_phase": "corners",
        **angle_meta,
    }
    if uniform_info is not None:
        opt_info["uniform_scale_factor"] = uniform_info.get("scale_factor")
    if uv_info is not None:
        opt_info["uv_scale_warm_start"] = True
        opt_info["scale_sx"] = uv_info["scale_sx"]
        opt_info["scale_sy"] = uv_info["scale_sy"]
        opt_info["uv_scale_nfev"] = uv_info["nfev"]
    meta = {
        "mode": "corners",
        "stop_phase": "corners",
        "exact": max_len_err < _CALIB_LENGTH_TOLERANCE_MM,
        "rigid_motion": rigid,
        "corners": corners,
        "corner_move": move,
        **angle_meta,
        **_solver_diagnostics_meta(
            rank, n_primary, include_angle, "corners",
            n_lengths=len(specs), n_angles=n_angles,
            include_centroid=include_centroid),
        "point_by_index": point_by_index or {},
    }
    if uniform_info is not None:
        meta["uniform_scale_factor"] = uniform_info.get("scale_factor")
    if uv_info is not None:
        meta["uv_scale_warm_start"] = True
        meta["scale_sx"] = uv_info["scale_sx"]
        meta["scale_sy"] = uv_info["scale_sy"]
        meta["uv_scale_success"] = uv_info["success"]
        meta["uv_scale_nfev"] = uv_info["nfev"]
    return corners, H, opt_info, meta
def _corner_movement_stats(params, params0):
    deltas = params - params0
    per_corner = [
        float(np.hypot(deltas[i], deltas[i + 1]))
        for i in range(0, len(params), 2)
    ]
    return {
        "rms_mm": float(np.sqrt(np.mean(np.square(deltas)))),
        "max_mm": float(max(per_corner)),
        "total_mm": float(np.sum(per_corner)),
    }
def compute_calibration_corners(lines, img):
    """Calibrate by moving quad corners (unified least_squares).

    Residuals: line lengths, sqrt(distortion energy), minimal rigid
    translation of the quad centroid. Secondary terms apply when
    lengths are feasible; they vanish in importance when length errors dominate.
    """
    if not lines:
        raise ValueError("Mindestens 1 Referenzlinie erforderlich")
    if not image_objects.is_aligned_image(img):
        raise ValueError("AlignedImage für Eckpunkt-Kalibrierung erforderlich")

    pa._ensure_warp_matrix(img)
    corners0 = pa._aligned_corners(img)
    params0 = hg._pack_corners_xy(corners0)
    z_vals = hg._corner_z_values(corners0)
    specs, endpoint_welds = _reference_line_specs(lines, img)

    if least_squares is None:
        raise RuntimeError("scipy is required for corner calibration")

    corners, H, opt_info, meta = _solve_corner_calibration(specs, params0, z_vals)
    meta["endpoint_welds"] = endpoint_welds
    meta["specs"] = specs
    return corners, H, opt_info, meta


def compute_calibration_from_specs(
        length_specs, img, constraints=None, line_by_geo=None, sketch=None,
        point_by_index=None):
    """Calibrate image corners from length + optional angle/point constraints."""
    if not length_specs:
        raise ValueError("Mindestens 1 Soll-Länge erforderlich")
    if not image_objects.is_aligned_image(img):
        raise ValueError("AlignedImage für Eckpunkt-Kalibrierung erforderlich")

    pa._ensure_warp_matrix(img)
    corners0 = pa._aligned_corners(img)
    params0 = hg._pack_corners_xy(corners0)
    z_vals = hg._corner_z_values(corners0)

    if least_squares is None:
        raise RuntimeError("scipy is required for corner calibration")

    corners, H, opt_info, meta = _solve_corner_calibration(
        length_specs, params0, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch,
        point_by_index=point_by_index)
    meta["specs"] = length_specs
    meta["constraints"] = constraints or {}
    return corners, H, opt_info, meta

def _reference_line_length(line):
    return float(image_objects.reference_line_length_xy(line))
def _predicted_line_length_spec(spec, H):
    return hg._line_length_uv(
        spec["u0"], spec["v0"], spec["u1"], spec["v1"], H)
def _print_scale_solver_debug(lines, H, specs, phase, opt_info=None, meta=None):
    after = phase.startswith("Nach")
    App.Console.PrintMessage(
        "\n[Scale Solver Debug] {} ({} Linie(n))\n".format(phase, len(lines)))
    if not after and meta is not None:
        _print_solver_diagnostics(meta=meta, opt_info=opt_info)
        welds = meta.get("endpoint_welds", 0)
        if welds:
            App.Console.PrintMessage(
                "  Endpunkt-Knoten: {} zusammengeführt (≤ {:.1f} mm)\n".format(
                    welds, REF_LINE_ENDPOINT_SNAP_MM))
        if opt_info is not None:
            res = opt_info.get("residuals")
            if res is not None:
                App.Console.PrintMessage(
                    "  Restfehler Längen [mm]: {}\n".format(
                        ", ".join("{:.4f}".format(r) for r in res)))
        if "C" in meta:
            C = meta["C"]
            App.Console.PrintMessage(
                "  C = [[{:.4f}, {:.4f}, {:.4f}], [{:.4f}, {:.4f}, {:.4f}], "
                "[{:.4f}, {:.4f}, 1]]\n".format(
                    C[0, 0], C[0, 1], C[0, 2],
                    C[1, 0], C[1, 1], C[1, 2],
                    C[2, 0], C[2, 1]))
    if after:
        App.Console.PrintMessage(
            "  {:<22} {:>11} {:>11} {:>11}\n".format(
                "Linie", "Ist 2D", "Soll", "Delta"))
    else:
        App.Console.PrintMessage(
            "  {:<22} {:>11} {:>11} {:>11} {:>11}\n".format(
                "Linie", "Ist [mm]", "Soll [mm]", "Modell [mm]", "Delta [mm]"))
    max_delta = 0.0
    for line, spec in zip(lines, specs):
        label = spec["label"]
        if after:
            current = _reference_line_length(line)
            target = spec["target"]
            delta = current - target
            max_delta = max(max_delta, abs(delta))
            App.Console.PrintMessage(
                "  {:<22} {:11.4f} {:11.4f} {:11.4f}\n".format(
                    label, current, target, delta))
        else:
            current = _reference_line_length(line)
            target = spec["target"]
            predicted = _predicted_line_length_spec(spec, H)
            delta = predicted - target
            max_delta = max(max_delta, abs(delta))
            App.Console.PrintMessage(
                "  {:<22} {:11.4f} {:11.4f} {:11.4f} {:11.4f}\n".format(
                    label, current, target, predicted, delta))
    summary = "max |Ist 2D - Soll|" if after else "max |Modell - Soll|"
    App.Console.PrintMessage(
        "  {} = {:.6f} mm\n".format(summary, max_delta))
def _constraint_line_index(item, legacy_key="geo"):
    if isinstance(item, dict):
        if "line" in item:
            return int(item["line"])
        if legacy_key in item:
            return int(item[legacy_key])
    return int(item)
def _constraint_line_pair(item):
    if isinstance(item, dict):
        la = item.get("line_a", item.get("geo_a"))
        lb = item.get("line_b", item.get("geo_b"))
        return int(la), int(lb)
    return int(item[0]), int(item[1])


def _constraint_point_index(item):
    if isinstance(item, dict):
        if "point" in item:
            return int(item["point"])
    return int(item)


def _has_fixed_point_constraints(constraints, point_by_index=None):
    items = (constraints or {}).get("fixed_points") or []
    if not items:
        return False
    if not point_by_index:
        return True
    return any(
        _constraint_point_index(item) in point_by_index
        for item in items
        if isinstance(item, dict))


def _point_by_index_from_points_meta(points_meta):
    out = {}
    for i, pt in enumerate(points_meta or []):
        key = int(pt.get("point", i))
        out[key] = pt
    return out


def _line_by_index_from_lines_meta(lines_meta):
    out = {}
    for i, line in enumerate(lines_meta):
        key = int(line.get("line", i))
        out[key] = line
    return out
def _normalize_constraints_lines(constraints, n_lines):
    """Store constraints by stable line index L0..L(n-1)."""
    out = image_calibration_objects.default_constraints()

    def valid(line_idx):
        return 0 <= int(line_idx) < n_lines

    for item in constraints.get("lengths", []):
        li = _constraint_line_index(item)
        if valid(li):
            out["lengths"].append({
                "line": int(li),
                "target_mm": float(item["target_mm"]),
            })
    for item in constraints.get("parallel", []):
        la, lb = _constraint_line_pair(item)
        if valid(la) and valid(lb):
            out["parallel"].append({"line_a": int(la), "line_b": int(lb)})
    for item in constraints.get("perpendicular", []):
        la, lb = _constraint_line_pair(item)
        if valid(la) and valid(lb):
            out["perpendicular"].append(
                {"line_a": int(la), "line_b": int(lb)})
    for item in constraints.get("horizontal", []):
        li = _constraint_line_index(item)
        if valid(li):
            out["horizontal"].append({"line": int(li)})
    for item in constraints.get("vertical", []):
        li = _constraint_line_index(item)
        if valid(li):
            out["vertical"].append({"line": int(li)})
    for item in constraints.get("fixed_points", []):
        pi = _constraint_point_index(item)
        if 0 <= int(pi):
            out["fixed_points"].append({
                "point": int(pi),
                "target_x_mm": float(item["target_x_mm"]),
                "target_y_mm": float(item["target_y_mm"]),
            })
    return out
def _remap_constraint_geos(constraints, geo_map):
    if not geo_map:
        return constraints

    def map_line(line_idx):
        line_idx = int(line_idx)
        return geo_map.get(line_idx, line_idx)

    out = image_calibration_objects.default_constraints()
    for item in constraints.get("lengths", []):
        out["lengths"].append({
            "line": map_line(_constraint_line_index(item)),
            "target_mm": float(item["target_mm"]),
        })
    for item in constraints.get("parallel", []):
        la, lb = _constraint_line_pair(item)
        out["parallel"].append({
            "line_a": map_line(la),
            "line_b": map_line(lb),
        })
    for item in constraints.get("perpendicular", []):
        la, lb = _constraint_line_pair(item)
        out["perpendicular"].append({
            "line_a": map_line(la),
            "line_b": map_line(lb),
        })
    for item in constraints.get("horizontal", []):
        out["horizontal"].append(
            {"line": map_line(_constraint_line_index(item))})
    for item in constraints.get("vertical", []):
        out["vertical"].append(
            {"line": map_line(_constraint_line_index(item))})
    for item in constraints.get("fixed_points", []):
        out["fixed_points"].append({
            "point": int(_constraint_point_index(item)),
            "target_x_mm": float(item["target_x_mm"]),
            "target_y_mm": float(item["target_y_mm"]),
        })
    return out
def _sketch_line_geometries(sketch):
    out = []
    for i, geom in enumerate(sketch.Geometry):
        if isinstance(geom, Part.LineSegment):
            out.append((i, geom))
    return out


def _constraints_need_geo_remap(constraints, sketch):
    n_lines = len(_sketch_line_geometries(sketch))
    if n_lines == 0:
        return False
    valid = set(range(n_lines))

    def missing_line(line_idx):
        return int(line_idx) not in valid

    for item in constraints.get("lengths", []):
        if missing_line(_constraint_line_index(item)):
            return True
    for key in ("parallel", "perpendicular"):
        for item in constraints.get(key, []):
            la, lb = _constraint_line_pair(item)
            if missing_line(la) or missing_line(lb):
                return True
    for key in ("horizontal", "vertical"):
        for item in constraints.get(key, []):
            if missing_line(_constraint_line_index(item)):
                return True
    return False
def _remap_constraints_to_sketch(constraints, sketch):
    """Match saved line ids to current sketch lines by order (legacy geo ids)."""
    n_lines = len(_sketch_line_geometries(sketch))
    if n_lines == 0 or not _constraints_need_geo_remap(constraints, sketch):
        return constraints

    old_lines = set()

    def collect(line_idx):
        old_lines.add(int(line_idx))

    for item in constraints.get("lengths", []):
        collect(_constraint_line_index(item))
    for key in ("parallel", "perpendicular"):
        for item in constraints.get(key, []):
            la, lb = _constraint_line_pair(item)
            collect(la)
            collect(lb)
    for key in ("horizontal", "vertical"):
        for item in constraints.get(key, []):
            collect(_constraint_line_index(item))

    old_sorted = sorted(old_lines)
    line_map = {}
    for i, old in enumerate(old_sorted):
        if i < n_lines:
            line_map[old] = i
    return _remap_constraint_geos(constraints, line_map)
def _length_specs_from_constraints(constraints, line_by_geo):
    specs = []
    for item in constraints.get("lengths", []):
        li = _constraint_line_index(item)
        if li not in line_by_geo:
            continue
        line = line_by_geo[li]
        specs.append({
            "label": line.get("label", "L{}".format(li)),
            "u0": line["u0"], "v0": line["v0"],
            "u1": line["u1"], "v1": line["v1"],
            "target": float(item["target_mm"]),
        })
    return specs
