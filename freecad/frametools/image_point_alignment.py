"""Point alignment: AlignedImage and homography overlay."""

import os

import FreeCAD as App
import FreeCADGui as Gui
import numpy as np

from . import image_objects
from . import image_homography as hg

compute_affine_2d = hg.compute_affine_2d
compute_homography = hg.compute_homography

def is_image_plane(obj):
    if obj is None:
        return False
    type_id = getattr(obj, "TypeId", "")
    return type_id == "Image::ImagePlane" or type_id.endswith(":ImagePlane")
def _linked_object(obj):
    if obj is None:
        return None
    try:
        if hasattr(obj, "getLinkedObject"):
            linked = obj.getLinkedObject()
            if linked is not None:
                return linked
    except Exception:
        pass
    return obj
def is_image_object(obj):
    obj = _linked_object(obj)
    if obj is None:
        return False
    if image_objects.is_aligned_image(obj):
        return True
    if is_image_plane(obj):
        return True
    if hasattr(obj, "ImageFile") and hasattr(obj, "Placement"):
        return True
    return False
def _aligned_corners(obj):
    """Quad corners at UV (0,0), (1,0), (1,1), (0,1)."""
    c0 = obj.Corner0
    cx = obj.CornerX
    cy = obj.CornerY
    if hasattr(obj, "Corner1"):
        c1 = obj.Corner1
    else:
        c1 = cx + cy - c0
    return c0, cx, c1, cy
def _image_file_from_plane(plane):
    if hasattr(plane, "ImageFile"):
        return plane.ImageFile
    return ""


def resolve_image_file_path(path, document=None):
    """Resolve ImageFile path (absolute, document-relative, or basename in doc dir)."""
    path = str(path or "")
    if not path:
        return ""
    if os.path.isfile(path):
        return os.path.normpath(path)
    if document is not None and getattr(document, "FileName", None):
        doc_dir = os.path.dirname(document.FileName)
        for candidate in (
                os.path.join(doc_dir, path),
                os.path.join(doc_dir, os.path.basename(path))):
            if os.path.isfile(candidate):
                return os.path.normpath(candidate)
    return ""


def ensure_aligned_image_file(obj):
    """Ensure AlignedImage.ImageFile points at an existing file after restore."""
    if not image_objects.is_aligned_image(obj):
        return ""
    doc = getattr(obj, "Document", None)
    resolved = resolve_image_file_path(getattr(obj, "ImageFile", ""), doc)
    if not resolved:
        source = getattr(obj, "SourceImage", None)
        if source is not None:
            resolved = resolve_image_file_path(
                _image_file_from_plane(source), doc)
    if resolved and str(getattr(obj, "ImageFile", "")) != resolved:
        obj.ImageFile = resolved
    return resolved or str(getattr(obj, "ImageFile", "") or "")


def refresh_aligned_image_view(obj):
    """Reload Coin texture / warp after document open or path fix."""
    if not image_objects.is_aligned_image(obj):
        return
    ensure_aligned_image_file(obj)
    _ensure_corner1_property(obj)
    _ensure_warp_matrix(obj)
    vobj = getattr(obj, "ViewObject", None)
    if vobj is None or not getattr(vobj, "Proxy", None):
        return
    vobj.Proxy.attach(vobj)


def _image_plane_local_corners(xsize, ysize):
    """Image::ImagePlane uses a centered local origin (see ViewProviderImagePlane)."""
    half_x = xsize / 2.0
    half_y = ysize / 2.0
    return (
        App.Vector(-half_x, -half_y, 0),
        App.Vector(half_x, -half_y, 0),
        App.Vector(half_x, half_y, 0),
        App.Vector(-half_x, half_y, 0),
    )
def _corners_from_image_plane(plane):
    xsize, ysize = _image_dimensions(plane)
    plm = plane.Placement
    corners = _image_plane_local_corners(xsize, ysize)
    return tuple(plm.multVec(c) for c in corners) + (_image_file_from_plane(plane),)
def create_aligned_image_from_plane(plane, hide_source=True):
    doc = plane.Document
    c0, cx, c1, cy, image_file = _corners_from_image_plane(plane)
    label = plane.Label
    base_name = label if label.endswith("_aligned") else "{}_aligned".format(label)
    name = doc.getUniqueObjectName(base_name.replace(" ", "_"))
    obj = doc.addObject("App::FeaturePython", name)
    image_objects.AlignedImage(obj)
    obj.ImageFile = image_file
    obj.Corner0 = c0
    obj.CornerX = cx
    obj.Corner1 = c1
    obj.CornerY = cy
    _sync_warp_from_corners(obj)
    image_objects.ViewProviderAlignedImage(obj.ViewObject)
    obj.SourceImage = plane
    if hide_source and hasattr(plane, "ViewObject"):
        plane.ViewObject.Visibility = False
    return obj
def _ensure_corner1_property(img):
    if not image_objects.is_aligned_image(img):
        return
    if "Corner1" in img.PropertiesList:
        return
    img.addProperty(
        "App::PropertyVector", "Corner1", "Image",
        "Ecke diagonal (UV 1,1)")
    img.Corner1 = img.CornerX + img.CornerY - img.Corner0
def _matrix_is_identity(m, tol=1e-9):
    return (
        abs(m.A11 - 1.0) < tol and abs(m.A22 - 1.0) < tol
        and abs(m.A33 - 1.0) < tol and abs(m.A44 - 1.0) < tol
        and abs(m.A12) < tol and abs(m.A13) < tol and abs(m.A14) < tol
        and abs(m.A21) < tol and abs(m.A23) < tol and abs(m.A24) < tol
        and abs(m.A31) < tol and abs(m.A32) < tol and abs(m.A34) < tol
        and abs(m.A41) < tol and abs(m.A42) < tol and abs(m.A43) < tol
    )
def _ensure_warp_matrix(img):
    if not image_objects.is_aligned_image(img):
        return
    if "WarpMatrix" not in img.PropertiesList:
        img.addProperty(
            "App::PropertyMatrix", "WarpMatrix", "Image",
            "Projektive Abbildung UV-Einheitsquad -> Welt")
        img.WarpMatrix = App.Matrix()
    corners = _aligned_corners(img)
    if _matrix_is_identity(img.WarpMatrix):
        span = max(
            corners[0].distanceToPoint(corners[1]),
            corners[0].distanceToPoint(corners[3]),
        )
        if span > 1e-3:
            _sync_warp_from_corners(img)
def find_aligned_image_for_source(img):
    """Return AlignedImage for a source object, without creating one."""
    if img is None:
        return None
    if image_objects.is_aligned_image(img):
        _ensure_corner1_property(img)
        _ensure_warp_matrix(img)
        return img
    doc = img.Document if hasattr(img, "Document") else None
    if doc is None:
        return None
    for obj in doc.Objects:
        if not image_objects.is_aligned_image(obj):
            continue
        if getattr(obj, "SourceImage", None) == img:
            _ensure_corner1_property(obj)
            _ensure_warp_matrix(obj)
            return obj
    return None


def _reset_aligned_from_source(aligned, source):
    """Restore AlignedImage corners from the source ImagePlane."""
    c0, cx, c1, cy, _ = _corners_from_image_plane(source)
    _restore_aligned_corners(aligned, (c0, cx, c1, cy))
    _sync_warp_from_corners(aligned)


def ensure_aligned_image(img):
    if image_objects.is_aligned_image(img):
        _ensure_corner1_property(img)
        _ensure_warp_matrix(img)
        ensure_aligned_image_file(img)
        return img
    existing = find_aligned_image_for_source(img)
    if existing is not None:
        return existing
    return create_aligned_image_from_plane(img)
def _corners_for_image(img):
    if image_objects.is_aligned_image(img):
        return _aligned_corners(img)
    c0, cx, c1, cy, _ = _corners_from_image_plane(img)
    return c0, cx, c1, cy
def _snapshot_corners(img):
    c0, cx, c1, cy = _corners_for_image(img)
    return App.Vector(c0), App.Vector(cx), App.Vector(c1), App.Vector(cy)
def _snapshot_aligned_state(img):
    _ensure_warp_matrix(img)
    return _snapshot_corners(img), App.Matrix(img.WarpMatrix)
def _restore_aligned_corners(img, corners):
    c0, cx, c1, cy = corners
    img.Corner0 = App.Vector(c0)
    img.CornerX = App.Vector(cx)
    if hasattr(img, "Corner1"):
        img.Corner1 = App.Vector(c1)
    img.CornerY = App.Vector(cy)
def _restore_aligned_state(img, state):
    corners, warp = state
    _restore_aligned_corners(img, corners)
    img.WarpMatrix = warp
def _image_identity(img):
    """Stable key for an ImagePlane / its linked AlignedImage."""
    if image_objects.is_aligned_image(img):
        src = getattr(img, "SourceImage", None)
        if src is not None:
            return "plane:{}".format(src.Name)
        return "aligned:{}".format(img.Name)
    return "plane:{}".format(img.Name)
def _unique_images_from_selection(sel):
    """Selected image objects, deduping ImagePlane + linked AlignedImage."""
    images = []
    seen = set()
    for obj in sel:
        if not is_image_object(obj):
            continue
        identity = _image_identity(obj)
        if identity in seen:
            continue
        seen.add(identity)
        images.append(obj)
    return images
def _point_on_image(point, img, tol=0.05):
    try:
        u, v = _uv_on_image(point, img)
        return -tol <= u <= 1.0 + tol and -tol <= v <= 1.0 + tol
    except ValueError:
        return False
def _identify_ref_mov_images(images, pair_objs):
    if len(images) != 2:
        raise ValueError("Genau 2 Bilder erwartet")
    img_a, img_b = images[0], images[1]
    ref_hits = {img_a: 0, img_b: 0}
    mov_hits = {img_a: 0, img_b: 0}
    for pair in pair_objs:
        for img in (img_a, img_b):
            if _point_on_image(pair.RefPoint, img):
                ref_hits[img] += 1
            if _point_on_image(pair.MovPoint, img):
                mov_hits[img] += 1

    score_a = ref_hits[img_a] - mov_hits[img_a]
    score_b = ref_hits[img_b] - mov_hits[img_b]
    if score_a > score_b:
        return img_a, img_b
    if score_b > score_a:
        return img_b, img_a
    if ref_hits[img_a] >= ref_hits[img_b]:
        return img_a, img_b
    return img_b, img_a
def _prepare_aligned_image_pair(sel, pair_objs=None):
    raw_images = _unique_images_from_selection(sel)
    if len(raw_images) != 2:
        return None, None
    if pair_objs:
        img_ref, img_mov = _identify_ref_mov_images(raw_images, pair_objs)
    else:
        img_ref, img_mov = raw_images[0], raw_images[1]
    return img_ref, ensure_aligned_image(img_mov)
def convert_selected_to_aligned_images():
    sel = Gui.Selection.getSelection()
    if not sel:
        App.Console.PrintError("Bildobjekt(e) auswählen.\n")
        return
    doc = App.ActiveDocument
    doc.openTransaction("Convert to Aligned Image")
    try:
        created = []
        skipped = 0
        for obj in sel:
            if image_objects.is_aligned_image(obj):
                App.Console.PrintMessage(
                    "{} ist bereits ein AlignedImage.\n".format(obj.Label))
                continue
            if not is_image_object(obj):
                skipped += 1
                continue
            created.append(create_aligned_image_from_plane(obj))
        doc.commitTransaction()
        doc.recompute()
        if created:
            App.Console.PrintMessage(
                "{} AlignedImage(s) erstellt.\n".format(len(created)))
        if skipped:
            App.Console.PrintMessage(
                "{} Nicht-Bildobjekt(e) übersprungen.\n".format(skipped))
    except Exception:
        doc.abortTransaction()
        raise
def _refresh_aligned_view(img):
    vobj = getattr(img, "ViewObject", None)
    if vobj and getattr(vobj, "Proxy", None):
        vobj.Proxy.updateData(img, "WarpMatrix")
    img.touch()
def _sync_warp_from_corners(img):
    c0, cx, c1, cy = _aligned_corners(img)
    H = hg._homography_from_corners(c0, cx, c1, cy)
    z_base = hg._z_base_from_corners(c0, cx, c1, cy)
    img.WarpMatrix = hg._homography_to_warp_matrix(H, z_base)
def _sync_corners_from_homography(img, H, z_base):
    for u, v, prop in (
            (0.0, 0.0, "Corner0"),
            (1.0, 0.0, "CornerX"),
            (1.0, 1.0, "Corner1"),
            (0.0, 1.0, "CornerY")):
        setattr(img, prop, hg._apply_homography_uv(u, v, H, z_base))
def _set_aligned_homography(img, H):
    c0, cx, c1, cy = _aligned_corners(img)
    z_base = hg._z_base_from_corners(c0, cx, c1, cy)
    img.WarpMatrix = hg._homography_to_warp_matrix(H, z_base)
    _sync_corners_from_homography(img, H, z_base)
def _uv_on_image(point, img):
    if image_objects.is_aligned_image(img):
        _ensure_warp_matrix(img)
        H = hg._homography_from_warp_matrix(img.WarpMatrix)
        return hg._uv_from_homography(point, H)
    c0, cx, c1, cy = _corners_for_image(img)
    return hg._uv_from_quad(point, c0, cx, c1, cy)
def _apply_homography_to_aligned(img, H):
    _set_aligned_homography(img, H)
def _predicted_line_length(line, M):
    return float(np.linalg.norm(M @ line_vector_2d(line)))
def _image_dimensions(img):
    if hasattr(img, "XSize"):
        return img.XSize.Value, img.YSize.Value
    if hasattr(img, "Width"):
        return img.Width.Value, img.Height.Value
    return 100.0, 100.0
def _set_image_dimensions(img, width, height):
    if hasattr(img, "XSize"):
        img.XSize = width
        img.YSize = height
    elif hasattr(img, "Width"):
        img.Width = width
        img.Height = height
def _transform_world_xy(point, M, t):
    return App.Vector(
        M[0, 0] * point.x + M[0, 1] * point.y + t[0],
        M[1, 0] * point.x + M[1, 1] * point.y + t[1],
        point.z,
    )
def _apply_affine_to_aligned(img, M, t):
    c0, cx, c1, cy = _aligned_corners(img)
    img.Corner0 = _transform_world_xy(c0, M, t)
    img.CornerX = _transform_world_xy(cx, M, t)
    if hasattr(img, "Corner1"):
        img.Corner1 = _transform_world_xy(c1, M, t)
    img.CornerY = _transform_world_xy(cy, M, t)
    _sync_warp_from_corners(img)
def _apply_symmetric_to_aligned(img, M, origin):
    _ensure_warp_matrix(img)
    H = hg._homography_from_warp_matrix(img.WarpMatrix)
    H_new = hg._compose_world_affine_homography(H, M, origin)
    _set_aligned_homography(img, H_new)
def _apply_affine_to_image(img, M, t):
    """Transform ImagePlane via corner mapping (XSize/YSize + Placement)."""
    xsize, ysize = _image_dimensions(img)
    plm = img.Placement
    bl, br, tr, tl = _image_plane_local_corners(xsize, ysize)
    center = App.Vector(0, 0, 0)

    bl_n = _transform_world_xy(plm.multVec(bl), M, t)
    br_n = _transform_world_xy(plm.multVec(br), M, t)
    tl_n = _transform_world_xy(plm.multVec(tl), M, t)
    center_n = _transform_world_xy(plm.multVec(center), M, t)

    x_axis = br_n - bl_n
    y_axis = tl_n - bl_n
    new_xsize = x_axis.Length
    new_ysize = y_axis.Length
    if new_xsize < 1e-9 or new_ysize < 1e-9:
        raise ValueError("Image size collapsed after transform")

    x_dir = App.Vector(x_axis)
    y_dir = App.Vector(y_axis)
    x_dir.normalize()
    y_dir.normalize()
    img.Placement = hg._placement_from_axes(center_n, x_dir, y_dir)
    _set_image_dimensions(img, new_xsize, new_ysize)
def apply_2d_transform(obj, M, t):
    if image_objects.is_aligned_image(obj):
        _apply_affine_to_aligned(obj, M, t)
        return
    if is_image_object(obj):
        _apply_affine_to_image(obj, M, t)
        return
    mat = App.Matrix()
    mat.A11, mat.A12 = M[0, 0], M[0, 1]
    mat.A21, mat.A22 = M[1, 0], M[1, 1]
    mat.A14, mat.A24 = t[0], t[1]
    mat.A33 = mat.A44 = 1.0
    T = App.Placement(mat)
    obj.Placement = T.multiply(obj.Placement)
def _map_local_point(p_local, M, o_local):
    rel = np.array([p_local.x - o_local.x, p_local.y - o_local.y])
    mapped = M @ rel
    return App.Vector(o_local.x + mapped[0], o_local.y + mapped[1], p_local.z)
def _apply_local_affine_to_image(img, M, origin):
    """Apply symmetric scale/shear in image-local coordinates."""
    xsize, ysize = _image_dimensions(img)
    plm = img.Placement
    o_local = plm.inverse().multVec(
        App.Vector(origin[0], origin[1], plm.Base.z))

    bl, br, tr, tl = _image_plane_local_corners(xsize, ysize)
    bl_w = plm.multVec(_map_local_point(bl, M, o_local))
    br_w = plm.multVec(_map_local_point(br, M, o_local))
    tl_w = plm.multVec(_map_local_point(tl, M, o_local))
    center_w = plm.multVec(_map_local_point(App.Vector(0, 0, 0), M, o_local))

    x_axis = br_w - bl_w
    y_axis = tl_w - bl_w
    new_xsize = x_axis.Length
    new_ysize = y_axis.Length
    if new_xsize < 1e-9 or new_ysize < 1e-9:
        raise ValueError("Image size collapsed after transform")

    x_dir = App.Vector(x_axis)
    y_dir = App.Vector(y_axis)
    x_dir.normalize()
    y_dir.normalize()
    img.Placement = hg._placement_from_axes(center_w, x_dir, y_dir)
    _set_image_dimensions(img, new_xsize, new_ysize)
def _transform_point_local(p, M, origin, plm):
    o_local = plm.inverse().multVec(
        App.Vector(origin[0], origin[1], plm.Base.z))
    p_local = plm.inverse().multVec(p)
    return plm.multVec(_map_local_point(p_local, M, o_local))
def _transform_point_world(p, M, origin):
    rel = np.array([p.x - origin[0], p.y - origin[1]])
    mapped = M @ rel
    return App.Vector(mapped[0] + origin[0], mapped[1] + origin[1], p.z)
def _object_uses_image_homography(obj, img):
    if obj == img:
        return True
    if image_objects.is_reference_line(obj):
        link = getattr(obj, "Image", None)
        return link is None or link == img
    if image_objects.is_feature_pair(obj):
        return True
    return False
def _snapshot_homography_targets(objects, img):
    from . import image_constraint_solver as cs

    snapshots = {}
    ref_lines = [
        o for o in objects
        if image_objects.is_reference_line(o) and o != img]
    welded, _ = cs._weld_reference_line_endpoint_uvs(ref_lines, img)
    for obj in objects:
        if obj == img:
            continue
        if image_objects.is_reference_line(obj):
            if (obj, "start") in welded:
                snapshots[obj] = {
                    "start": welded[(obj, "start")],
                    "end": welded[(obj, "end")],
                }
            else:
                snapshots[obj] = {
                    "start": _uv_on_image(obj.Start, img),
                    "end": _uv_on_image(obj.End, img),
                }
        elif image_objects.is_feature_pair(obj):
            snapshots[obj] = {
                "mov": _uv_on_image(obj.MovPoint, img),
            }
    return snapshots
def apply_corner_calibration(img, corners, objects):
    """Set quad corners and re-place objects via fixed UV."""
    _ensure_warp_matrix(img)
    snapshots = _snapshot_homography_targets(objects, img)
    corners = tuple(App.Vector(c) for c in corners)
    _restore_aligned_corners(img, corners)
    _sync_warp_from_corners(img)
    for obj, snap in snapshots.items():
        if image_objects.is_reference_line(obj):
            u0, v0 = snap["start"]
            u1, v1 = snap["end"]
            obj.Start = _world_on_image_uv(u0, v0, img)
            obj.End = _world_on_image_uv(u1, v1, img)
        elif image_objects.is_feature_pair(obj):
            u, v = snap["mov"]
            obj.MovPoint = _world_on_image_uv(u, v, img)
def apply_homography_calibration(img, H_new, objects):
    """Set WarpMatrix to H_new and re-place objects via fixed UV."""
    _ensure_warp_matrix(img)
    snapshots = _snapshot_homography_targets(objects, img)
    _set_aligned_homography(img, H_new)
    for obj, snap in snapshots.items():
        if image_objects.is_reference_line(obj):
            u0, v0 = snap["start"]
            u1, v1 = snap["end"]
            obj.Start = _world_on_image_uv(u0, v0, img)
            obj.End = _world_on_image_uv(u1, v1, img)
        elif image_objects.is_feature_pair(obj):
            u, v = snap["mov"]
            obj.MovPoint = _world_on_image_uv(u, v, img)
def apply_scale_via_homography(img, M, origin, objects):
    """Legacy: compose affine M with current H."""
    _ensure_warp_matrix(img)
    H = hg._homography_from_warp_matrix(img.WarpMatrix)
    H_new = hg._compose_world_affine_homography(H, M, origin)
    apply_homography_calibration(img, H_new, objects)
def apply_scale_matrix(obj, origin, M, mov_only=False):
    if image_objects.is_aligned_image(obj):
        _apply_symmetric_to_aligned(obj, M, origin)
    elif is_image_object(obj):
        _apply_local_affine_to_image(obj, M, origin)
    elif image_objects.is_feature_pair(obj):
        if not mov_only:
            obj.RefPoint = _transform_point_world(obj.RefPoint, M, origin)
        obj.MovPoint = _transform_point_world(obj.MovPoint, M, origin)
    elif image_objects.is_reference_line(obj):
        obj.Start = _transform_point_world(obj.Start, M, origin)
        obj.End = _transform_point_world(obj.End, M, origin)
    else:
        _apply_scale_shear_placement(obj, origin, M)
def _apply_scale_shear_placement(obj, origin, M):
    mat = App.Matrix()
    mat.A11, mat.A12 = M[0, 0], M[0, 1]
    mat.A21, mat.A22 = M[1, 0], M[1, 1]
    mat.A33 = mat.A44 = 1.0
    T_scale = App.Placement(mat)
    T_origin = App.Placement(App.Vector(origin[0], origin[1], 0), App.Rotation())
    full = T_origin.multiply(T_scale).multiply(T_origin.inverse())
    obj.Placement = full.multiply(obj.Placement)
def line_vector_2d(line):
    p1 = line.Start
    p2 = line.End
    return np.array([p2.x - p1.x, p2.y - p1.y])
def line_start_2d(line):
    return np.array([line.Start.x, line.Start.y])
def pairs_from_objects(objects):
    pairs = []
    for obj in objects:
        if image_objects.is_feature_pair(obj):
            image_objects.ensure_feature_pair(obj)
            pairs.append((obj.RefPoint, obj.MovPoint))
    return pairs
def feature_pairs_from_selection(sel):
    out = []
    for o in sel:
        if image_objects.is_feature_pair(o):
            image_objects.ensure_feature_pair(o)
            out.append(o)
    return out
def reference_lines_from_selection(sel):
    lines = []
    for obj in sel:
        if image_objects.is_reference_line(obj):
            image_objects.ensure_reference_line(obj)
            if obj.TargetLength.Value > 0:
                lines.append(obj)
    return lines
def images_from_selection(sel):
    return [o for o in sel if is_image_object(o)]
def _world_on_image_uv(u, v, img):
    c0, cx, c1, cy = _corners_for_image(img)
    z = hg._bilinear_z(u, v, c0, cx, c1, cy)
    if image_objects.is_aligned_image(img):
        _ensure_warp_matrix(img)
        H = hg._homography_from_warp_matrix(img.WarpMatrix)
    else:
        H = hg._homography_from_corners(c0, cx, c1, cy)
    return hg._apply_homography_uv(u, v, H, z)
def project_point_to_image(point, img):
    if img is None:
        return App.Vector(point)
    if image_objects.is_aligned_image(img):
        _ensure_corner1_property(img)
        _ensure_warp_matrix(img)
    uv = _uv_on_image(point, img)
    return _world_on_image_uv(uv[0], uv[1], img)
def _reference_image_from_selection():
    sel = Gui.Selection.getSelection() if Gui else []
    images = _unique_images_from_selection(sel)
    if not images:
        return None
    img = images[0]
    if len(images) > 1:
        aligned = [o for o in images if image_objects.is_aligned_image(o)]
        if aligned:
            img = aligned[0]
    if image_objects.is_aligned_image(img):
        _ensure_corner1_property(img)
        _ensure_warp_matrix(img)
        return img
    return ensure_aligned_image(img)
def _guess_image_for_line(obj):
    doc = obj.Document
    if doc is None:
        return None
    aligned = [
        o for o in doc.Objects if image_objects.is_aligned_image(o)]
    if len(aligned) == 1:
        return aligned[0]
    return None


def overlay_images():
    sel = Gui.Selection.getSelection()
    raw_images = _unique_images_from_selection(sel)
    pair_objs = feature_pairs_from_selection(sel)

    if len(raw_images) != 2:
        App.Console.PrintError(
            "Genau 2 Bildobjekte auswählen (Referenz + Bild 2).\n")
        return
    if len(pair_objs) < 3:
        App.Console.PrintError("Mindestens 3 Feature-Paare auswählen.\n")
        return

    doc = App.ActiveDocument
    doc.openTransaction("Overlay Images")
    try:
        img_ref_raw, img_mov_raw = _identify_ref_mov_images(
            raw_images, pair_objs)
        if img_ref_raw == img_mov_raw:
            App.Console.PrintError(
                "Referenzbild und Bild 2 müssen unterschiedlich sein.\n")
            doc.abortTransaction()
            return

        ref_state = (
            _snapshot_aligned_state(img_ref_raw)
            if image_objects.is_aligned_image(img_ref_raw) else None)

        img_ref = img_ref_raw
        img_mov = ensure_aligned_image(img_mov_raw)

        App.Console.PrintMessage(
            "Referenz: {} ({}, fix) — Bild 2: {} (wird ausgerichtet)\n".format(
                img_ref.Label,
                "AlignedImage" if image_objects.is_aligned_image(img_ref)
                else "ImagePlane",
                img_mov.Label))

        pairs = pairs_from_objects(pair_objs)
        uv_world_pairs = []
        uv_coords = []
        for pair_obj in pair_objs:
            u, v = _uv_on_image(pair_obj.MovPoint, img_mov)
            uv_coords.append((u, v))
            uv_world_pairs.append(((u, v), pair_obj.RefPoint))

        H = compute_homography(uv_world_pairs)
        _set_aligned_homography(img_mov, H)
        _refresh_aligned_view(img_mov)
        z_base = hg._z_base_from_warp_matrix(img_mov.WarpMatrix)
        for pair_obj, (u, v) in zip(pair_objs, uv_coords):
            pair_obj.MovPoint = hg._apply_homography_uv(u, v, H, z_base)

        if ref_state is not None:
            _restore_aligned_state(img_ref, ref_state)

        rms = hg._rms_homography_error(uv_world_pairs, H)
        doc.commitTransaction()
        doc.recompute()
        App.Console.PrintMessage(
            "Bilder ausgerichtet (Homographie). RMS-Fehler: {:.4f} mm\n".format(
                rms))
    except Exception:
        doc.abortTransaction()
        raise
