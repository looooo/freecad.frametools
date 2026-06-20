"""Homography and UV coordinate math."""

import FreeCAD as App
import numpy as np

def _corner_vec(corner):
    return np.array([corner.x, corner.y, corner.z])
def _bilinear_z(u, v, c0, cx, c1, cy):
    return (
        (1.0 - u) * (1.0 - v) * c0.z
        + u * (1.0 - v) * cx.z
        + u * v * c1.z
        + (1.0 - u) * v * cy.z
    )
def _barycentric_triangle_3d(point, a, b, c):
    """Barycentric weights for *a*, *b*, *c* at *point* (3D, planar triangle)."""
    v0 = _corner_vec(b) - _corner_vec(a)
    v1 = _corner_vec(c) - _corner_vec(a)
    v2 = _corner_vec(point) - _corner_vec(a)
    d00 = np.dot(v0, v0)
    d01 = np.dot(v0, v1)
    d11 = np.dot(v1, v1)
    d20 = np.dot(v2, v0)
    d21 = np.dot(v2, v1)
    denom = d00 * d11 - d01 * d01
    if abs(denom) < 1e-18:
        return None
    w_b = (d11 * d20 - d01 * d21) / denom
    w_c = (d00 * d21 - d01 * d20) / denom
    w_a = 1.0 - w_b - w_c
    return w_a, w_b, w_c
def _uv_from_triangle(point, a, b, c, uv_a, uv_b, uv_c, tol=1e-5):
    weights = _barycentric_triangle_3d(point, a, b, c)
    if weights is None:
        return None
    w_a, w_b, w_c = weights
    if min(w_a, w_b, w_c) < -tol:
        return None
    u = w_a * uv_a[0] + w_b * uv_b[0] + w_c * uv_c[0]
    v = w_a * uv_a[1] + w_b * uv_b[1] + w_c * uv_c[1]
    return float(u), float(v)
def _uv_from_quad(point, c0, cx, c1, cy, tol=1e-5):
    """UV on the rendered image quad (Coin splits the quad into two triangles)."""
    hit = _uv_from_triangle(
        point, c0, cx, c1, (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), tol)
    if hit is not None:
        return hit
    hit = _uv_from_triangle(
        point, c0, c1, cy, (0.0, 0.0), (1.0, 1.0), (0.0, 1.0), tol)
    if hit is not None:
        return hit

    # Fallback: nearest triangle (click slightly off the surface numerically)
    best = None
    best_dist = None
    for tri, uvs in (
            ((c0, cx, c1), ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))),
            ((c0, c1, cy), ((0.0, 0.0), (1.0, 1.0), (0.0, 1.0)))):
        weights = _barycentric_triangle_3d(point, *tri)
        if weights is None:
            continue
        w_a, w_b, w_c = weights
        w_a = max(w_a, 0.0)
        w_b = max(w_b, 0.0)
        w_c = max(w_c, 0.0)
        w_sum = w_a + w_b + w_c
        if w_sum < 1e-12:
            continue
        w_a, w_b, w_c = w_a / w_sum, w_b / w_sum, w_c / w_sum
        uv_a, uv_b, uv_c = uvs
        u = w_a * uv_a[0] + w_b * uv_b[0] + w_c * uv_c[0]
        v = w_a * uv_a[1] + w_b * uv_b[1] + w_c * uv_c[1]
        pos = (
            w_a * _corner_vec(tri[0])
            + w_b * _corner_vec(tri[1])
            + w_c * _corner_vec(tri[2]))
        dist = np.linalg.norm(pos - _corner_vec(point))
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best = (u, v)
    if best is not None:
        return best
    raise ValueError("UV-Bestimmung auf Bild-Quad fehlgeschlagen")
def compute_affine_2d(pairs):
    N = len(pairs)
    A = np.zeros((2 * N, 6))
    b = np.zeros(2 * N)
    for i, (p_ref, p_mov) in enumerate(pairs):
        A[2 * i] = [p_mov.x, p_mov.y, 0, 0, 1, 0]
        A[2 * i + 1] = [0, 0, p_mov.x, p_mov.y, 0, 1]
        b[2 * i] = p_ref.x
        b[2 * i + 1] = p_ref.y
    params, *_ = np.linalg.lstsq(A, b, rcond=None)
    M = params[:4].reshape(2, 2)
    t = params[4:]
    return M, t
def compute_homography(uv_world_pairs):
    """Solve 2D homography (8 DOF, h33=1) mapping (u,v) -> (x,y).

    uv_world_pairs: iterable of ((u, v), RefPoint).
    3 points: underdetermined, minimum-norm exact fit.
    4 points: unique exact fit (non-degenerate).
    5+ points: least-squares fit.
    """
    pairs = list(uv_world_pairs)
    n = len(pairs)
    if n < 3:
        raise ValueError("Mindestens 3 Punktpaare für Homographie erforderlich")
    A = np.zeros((2 * n, 8))
    b = np.zeros(2 * n)
    for i, ((u, v), p_ref) in enumerate(pairs):
        x, y = p_ref.x, p_ref.y
        A[2 * i] = [u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v]
        A[2 * i + 1] = [0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v]
        b[2 * i] = x
        b[2 * i + 1] = y
    params, *_ = np.linalg.lstsq(A, b, rcond=None)
    return np.array([
        [params[0], params[1], params[2]],
        [params[3], params[4], params[5]],
        [params[6], params[7], 1.0],
    ])
def _apply_homography_uv(u, v, H, z=0.0):
    p = H @ np.array([u, v, 1.0])
    w = p[2]
    if abs(w) < 1e-12:
        raise ValueError("Homographie singulär")
    return App.Vector(float(p[0] / w), float(p[1] / w), z)
def _homography_from_corners(c0, cx, c1, cy):
    return compute_homography([
        ((0.0, 0.0), c0),
        ((1.0, 0.0), cx),
        ((1.0, 1.0), c1),
        ((0.0, 1.0), cy),
    ])
def _z_base_from_corners(c0, cx, c1, cy):
    return 0.25 * (c0.z + cx.z + c1.z + cy.z)
def _homography_to_warp_matrix(H, z_base=0.0):
    """FreeCAD Matrix: x'=A11*u+A12*v+A14, perspektiv in Zeile 4."""
    h11, h12, h13 = H[0, 0], H[0, 1], H[0, 2]
    h21, h22, h23 = H[1, 0], H[1, 1], H[1, 2]
    h31, h32 = H[2, 0], H[2, 1]
    mat = App.Matrix()
    mat.A11, mat.A12, mat.A13, mat.A14 = h11, h12, 0.0, h13
    mat.A21, mat.A22, mat.A23, mat.A24 = h21, h22, 0.0, h23
    mat.A31, mat.A32, mat.A33, mat.A34 = (
        h31 * z_base, h32 * z_base, 0.0, z_base)
    mat.A41, mat.A42, mat.A43, mat.A44 = h31, h32, 0.0, 1.0
    return mat
def _homography_from_warp_matrix(warp):
    return np.array([
        [warp.A11, warp.A12, warp.A14],
        [warp.A21, warp.A22, warp.A24],
        [warp.A41, warp.A42, warp.A44],
    ])
def _z_base_from_warp_matrix(warp):
    return warp.A34
def _uv_from_homography(point, H):
    Hinv = np.linalg.inv(H)
    p = Hinv @ np.array([point.x, point.y, 1.0])
    w = p[2]
    if abs(w) < 1e-12:
        raise ValueError("Homographie singulär")
    return float(p[0] / w), float(p[1] / w)
def _rms_homography_error(uv_world_pairs, H):
    errors = []
    for (u, v), p_ref in uv_world_pairs:
        q = _apply_homography_uv(u, v, H)
        errors.append(
            np.linalg.norm(
                np.array([q.x, q.y]) - np.array([p_ref.x, p_ref.y])))
    return float(np.sqrt(np.mean(np.square(errors))))
def _homography_from_params(params):
    return np.array([
        [params[0], params[1], params[2]],
        [params[3], params[4], params[5]],
        [params[6], params[7], 1.0],
    ])
def _line_length_uv(u0, v0, u1, v1, H):
    p0 = H @ np.array([u0, v0, 1.0])
    p1 = H @ np.array([u1, v1, 1.0])
    w0, w1 = p0[2], p1[2]
    if abs(w0) < 1e-12 or abs(w1) < 1e-12:
        raise ValueError("Homographie singulär")
    x0, y0 = p0[0] / w0, p0[1] / w0
    x1, y1 = p1[0] / w1, p1[1] / w1
    return float(np.hypot(x1 - x0, y1 - y0))


def _length_residuals_for_specs(specs, H):
    return [
        _line_length_uv(sp["u0"], sp["v0"], sp["u1"], sp["v1"], H) - sp["target"]
        for sp in specs
    ]


def _direction_xy_from_uv_line(u0, v0, u1, v1, H):
    p0 = H @ np.array([u0, v0, 1.0])
    p1 = H @ np.array([u1, v1, 1.0])
    w0, w1 = p0[2], p1[2]
    if abs(w0) < 1e-12 or abs(w1) < 1e-12:
        return np.array([0.0, 0.0])
    dx = float(p1[0] / w1 - p0[0] / w0)
    dy = float(p1[1] / w1 - p0[1] / w0)
    n = np.hypot(dx, dy)
    if n < 1e-12:
        return np.array([0.0, 0.0])
    return np.array([dx / n, dy / n])


def _parallel_sin_xy(da, db):
    return float(da[0] * db[1] - da[1] * db[0])


def _image_u_axis_xy(H):
    return _direction_xy_from_uv_line(0.0, 0.0, 1.0, 0.0, H)


def _image_v_axis_xy(H):
    return _direction_xy_from_uv_line(0.0, 0.0, 0.0, 1.0, H)


def _pack_corners_xy(corners):
    c0, cx, c1, cy = corners
    return np.array([c0.x, c0.y, cx.x, cx.y, c1.x, c1.y, cy.x, cy.y])
def _corner_z_values(corners):
    c0, cx, c1, cy = corners
    return (c0.z, cx.z, c1.z, cy.z)
def _corners_from_xy_params(params, z_vals):
    z0, zx, z1, zy = z_vals
    return (
        App.Vector(float(params[0]), float(params[1]), z0),
        App.Vector(float(params[2]), float(params[3]), zx),
        App.Vector(float(params[4]), float(params[5]), z1),
        App.Vector(float(params[6]), float(params[7]), zy),
    )
def _homography_from_xy_params(params, z_vals):
    return _homography_from_corners(*_corners_from_xy_params(params, z_vals))
def _length_residuals_corner_params(params, specs, z_vals, H=None):
    if H is None:
        H = _homography_from_xy_params(params, z_vals)
    return _length_residuals_for_specs(specs, H)
def _symmetric_matrix(sx, sy, sh):
    return np.array([[sx, sh], [sh, sy]])
def _compose_world_affine_homography(H, M, origin):
    """Apply world affine (no perspective) on the left: H' = A @ H."""
    ox, oy = float(origin[0]), float(origin[1])
    tx = ox - M[0, 0] * ox - M[0, 1] * oy
    ty = oy - M[1, 0] * ox - M[1, 1] * oy
    A = np.array([
        [M[0, 0], M[0, 1], tx],
        [M[1, 0], M[1, 1], ty],
        [0.0, 0.0, 1.0],
    ])
    return A @ H
def _placement_from_axes(origin, x_axis, y_axis):
    z_axis = x_axis.cross(y_axis)
    if z_axis.Length < 1e-9:
        z_axis = App.Vector(0, 0, 1)
    z_axis.normalize()
    mat = App.Matrix()
    mat.A11, mat.A21, mat.A31 = x_axis.x, x_axis.y, x_axis.z
    mat.A12, mat.A22, mat.A32 = y_axis.x, y_axis.y, y_axis.z
    mat.A13, mat.A23, mat.A33 = z_axis.x, z_axis.y, z_axis.z
    mat.A14, mat.A24, mat.A34 = origin.x, origin.y, origin.z
    return App.Placement(mat)
