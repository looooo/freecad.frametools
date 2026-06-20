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
_CALIB_RIGID_TRANSLATION_TOLERANCE_MM = 1.0
_CALIB_ANGLE_TOLERANCE_RAD = np.sin(np.deg2rad(1.0))
_CALIB_AXIS_ALIGNMENT_WEIGHT = 25.0
_CALIB_SOLVER_FTOL = 1e-6
_CALIB_SOLVER_XTOL = 1e-6
_CALIB_SOLVER_GTOL = 1e-6
_CALIB_SOLVER_MAX_NFEV = 250
_CALIB_SOLVER_REFINE_FTOL = 1e-9
_CALIB_SOLVER_REFINE_MAX_NFEV = 100


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
def _distortion_energy(params, params0, z_vals):
    """Zero for uniform scale; positive for anisotropic stretch or shear."""
    E0 = _quad_edge_basis(params0, z_vals)
    E1 = _quad_edge_basis(params, z_vals)
    det0 = np.linalg.det(E0)
    if abs(det0) < 1e-12:
        return 1e12
    F = E1 @ np.linalg.inv(E0)
    det_f = float(np.linalg.det(F))
    if det_f <= 1e-12:
        return 1e12
    sigma = np.sqrt(det_f)
    Fn = F / sigma
    return float(np.sum(np.square(Fn - np.eye(2))))
def _calibration_residuals(
        params, specs, params0, z_vals, constraints=None, line_by_geo=None,
        sketch=None):
    H = hg._homography_from_xy_params(params, z_vals)
    length_res = np.asarray(_length_residuals_for_specs(specs, H), dtype=float)
    length_part = length_res / _CALIB_LENGTH_TOLERANCE_MM
    E = _distortion_energy(params, params0, z_vals)
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
                * _CALIB_AXIS_ALIGNMENT_WEIGHT)
    parts.append(np.array([np.sqrt(max(E, 0.0))], dtype=float))
    parts.append(rigid_part)
    return np.concatenate(parts)


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
            deg = abs(np.degrees(np.arcsin(np.clip(sin_a, -1.0, 1.0))))
            App.Console.PrintMessage(
                "  {} L{}: Abweichung {:.3f}°\n".format(label, g, deg))


def _solve_corner_calibration(
        specs, params0, z_vals, constraints=None, line_by_geo=None,
        sketch=None):
    if least_squares is None:
        raise RuntimeError("scipy is required for corner calibration")

    residual_fn = lambda params: _calibration_residuals(
        params, specs, params0, z_vals,
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch)

    result = least_squares(
        residual_fn,
        params0.copy(),
        ftol=_CALIB_SOLVER_FTOL,
        xtol=_CALIB_SOLVER_XTOL,
        gtol=_CALIB_SOLVER_GTOL,
        max_nfev=_CALIB_SOLVER_MAX_NFEV)

    params = result.x
    corners = hg._corners_from_xy_params(params, z_vals)
    H = hg._homography_from_corners(*corners)
    length_res = _length_residuals_for_specs(specs, H)
    max_len_err = max((abs(r) for r in length_res), default=0.0)

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
        corners = hg._corners_from_xy_params(params, z_vals)
        H = hg._homography_from_corners(*corners)
        length_res = _length_residuals_for_specs(specs, H)
        max_len_err = max((abs(r) for r in length_res), default=0.0)

    E_dist = _distortion_energy(params, params0, z_vals)
    rigid = _rigid_motion_stats(params, params0, z_vals)
    move = _corner_movement_stats(params, params0)
    opt_info = {
        "cost": result.cost,
        "success": result.success,
        "residuals": length_res,
        "distortion_energy": E_dist,
        "rigid_motion": rigid,
        "corner_move": move,
    }
    meta = {
        "mode": "corners",
        "exact": max_len_err < _CALIB_LENGTH_TOLERANCE_MM,
        "distortion_energy": E_dist,
        "rigid_motion": rigid,
        "corners": corners,
        "corner_move": move,
    }
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
        length_specs, img, constraints=None, line_by_geo=None, sketch=None):
    """Calibrate image corners from length + optional angle constraints."""
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
        constraints=constraints, line_by_geo=line_by_geo, sketch=sketch)
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
        mode = meta.get("mode", "homography")
        mode_labels = {
            "corners": (
                "Eckpunkt-Optimierung — Längen, Verzerrungsenergie, "
                "min. Translation (einheitliches least_squares)"),
        }
        App.Console.PrintMessage(
            "  Modus: {}\n".format(mode_labels.get(mode, mode)))
        if mode == "corners":
            App.Console.PrintMessage(
                "  Verzerrungsenergie E={:.6e} (0 = reine Skalierung)\n".format(
                    meta.get("distortion_energy", 0.0)))
            rigid = meta.get("rigid_motion", {})
            App.Console.PrintMessage(
                "  Starre Bewegung: Δ={:.4f} mm, θ={:.4f}°\n".format(
                    rigid.get("translation_mm", 0.0),
                    rigid.get("rotation_deg", 0.0)))
            move = meta.get("corner_move", {})
            App.Console.PrintMessage(
                "  Längen exakt (< {:.3f} mm): {}\n".format(
                    _CALIB_LENGTH_TOLERANCE_MM, meta.get("exact", False)))
            welds = meta.get("endpoint_welds", 0)
            if welds:
                App.Console.PrintMessage(
                    "  Endpunkt-Knoten: {} zusammengeführt (≤ {:.1f} mm)\n".format(
                        welds, REF_LINE_ENDPOINT_SNAP_MM))
            App.Console.PrintMessage(
                "  Eck-Verschiebung: max={:.4f} mm, rms={:.4f} mm\n".format(
                    move.get("max_mm", 0.0), move.get("rms_mm", 0.0)))
        elif "C" in meta:
            C = meta["C"]
            App.Console.PrintMessage(
                "  C = [[{:.4f}, {:.4f}, {:.4f}], [{:.4f}, {:.4f}, {:.4f}], "
                "[{:.4f}, {:.4f}, 1]]\n".format(
                    C[0, 0], C[0, 1], C[0, 2],
                    C[1, 0], C[1, 1], C[1, 2],
                    C[2, 0], C[2, 1]))
        if opt_info is not None:
            App.Console.PrintMessage(
                "  Optimierung: cost={:.6e}, success={}\n".format(
                    opt_info.get("cost", 0.0), opt_info.get("success", True)))
            res = opt_info.get("residuals")
            if res is not None:
                App.Console.PrintMessage(
                    "  Restfehler Solver: {}\n".format(
                        ", ".join("{:.4f}".format(r) for r in res)))
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
