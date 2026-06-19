import os

import FreeCAD as App
import FreeCADGui as Gui
import numpy as np
from PySide import QtGui

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None

from . import image_objects


def _int_enum(value):
    try:
        return int(value)
    except TypeError:
        return value


def _spinbox_value(spinbox):
    return spinbox.property("rawValue")


def is_image_plane(obj):
    if obj is None:
        return False
    return obj.TypeId == "Image::ImagePlane"


def is_image_object(obj):
    if obj is None:
        return False
    if image_objects.is_aligned_image(obj):
        return True
    if is_image_plane(obj):
        return True
    return hasattr(obj, "ImageFile")


def _aligned_corners(obj):
    return obj.Corner0, obj.CornerX, obj.CornerY


def _barycentric_coords(point, c0, c1, c3):
    vx = np.array([c1.x - c0.x, c1.y - c0.y])
    vy = np.array([c3.x - c0.x, c3.y - c0.y])
    det = vx[0] * vy[1] - vx[1] * vy[0]
    if abs(det) < 1e-12:
        raise ValueError("Bild-Ecken kollinear")
    rel = np.array([point.x - c0.x, point.y - c0.y])
    uv = np.linalg.solve(np.column_stack([vx, vy]), rel)
    return uv[0], uv[1]


def _point_from_barycentric(u, v, c0, c1, c3):
    return App.Vector(
        c0.x + u * (c1.x - c0.x) + v * (c3.x - c0.x),
        c0.y + u * (c1.y - c0.y) + v * (c3.y - c0.y),
        c0.z + u * (c1.z - c0.z) + v * (c3.z - c0.z))


def _image_file_from_plane(plane):
    if hasattr(plane, "ImageFile"):
        return plane.ImageFile
    return ""


def _corners_from_image_plane(plane):
    xsize, ysize = _image_dimensions(plane)
    plm = plane.Placement
    c0 = plm.multVec(App.Vector(0, 0, 0))
    c1 = plm.multVec(App.Vector(xsize, 0, 0))
    c3 = plm.multVec(App.Vector(0, ysize, 0))
    return c0, c1, c3, _image_file_from_plane(plane)


def create_aligned_image_from_plane(plane, hide_source=True):
    doc = plane.Document
    c0, c1, c3, image_file = _corners_from_image_plane(plane)
    label = plane.Label
    base_name = label if label.endswith("_aligned") else "{}_aligned".format(label)
    name = doc.getUniqueObjectName(base_name.replace(" ", "_"))
    obj = doc.addObject("App::FeaturePython", name)
    image_objects.AlignedImage(obj)
    obj.ImageFile = image_file
    obj.Corner0 = c0
    obj.CornerX = c1
    obj.CornerY = c3
    image_objects.ViewProviderAlignedImage(obj.ViewObject)
    obj.SourceImage = plane
    if hide_source and hasattr(plane, "ViewObject"):
        plane.ViewObject.Visibility = False
    return obj


def ensure_aligned_image(img):
    if image_objects.is_aligned_image(img):
        return img
    doc = img.Document
    for obj in doc.Objects:
        if not image_objects.is_aligned_image(obj):
            continue
        src = getattr(obj, "SourceImage", None)
        if src and src == img:
            return obj
    aligned = create_aligned_image_from_plane(img)
    return aligned


def _corners_for_image(img):
    if image_objects.is_aligned_image(img):
        return _aligned_corners(img)
    c0, c1, c3, _ = _corners_from_image_plane(img)
    return c0, c1, c3


def _snapshot_corners(img):
    c0, c1, c3 = _corners_for_image(img)
    return App.Vector(c0), App.Vector(c1), App.Vector(c3)


def _restore_aligned_corners(img, corners):
    c0, c1, c3 = corners
    img.Corner0 = App.Vector(c0)
    img.CornerX = App.Vector(c1)
    img.CornerY = App.Vector(c3)


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
        c0, c1, c3 = _corners_for_image(img)
        u, v = _barycentric_coords(point, c0, c1, c3)
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


def _symmetric_matrix(sx, sy, sh):
    return np.array([[sx, sh], [sh, sy]])


def compute_scale_shear(lines):
    """Solve scale + shear with zero rotation (symmetric 2×2 matrix)."""
    if least_squares is None:
        raise RuntimeError("scipy is required for scale/shear calibration")

    def residuals(params):
        sx, sy, sh = params
        M = _symmetric_matrix(sx, sy, sh)
        res = []
        for line in lines:
            v = line_vector_2d(line)
            L = line.TargetLength.Value
            res.append(np.linalg.norm(M @ v) - L)
        return res

    result = least_squares(residuals, x0=[1.0, 1.0, 0.0])
    return result.x[0], result.x[1], result.x[2]


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


def _apply_affine_to_aligned(img, M, t):
    c0, c1, c3 = _aligned_corners(img)
    img.Corner0 = _transform_world_xy(c0, M, t)
    img.CornerX = _transform_world_xy(c1, M, t)
    img.CornerY = _transform_world_xy(c3, M, t)


def _apply_symmetric_to_aligned(img, M, origin):
    c0, c1, c3 = _aligned_corners(img)
    img.Corner0 = _transform_point_world(c0, M, origin)
    img.CornerX = _transform_point_world(c1, M, origin)
    img.CornerY = _transform_point_world(c3, M, origin)


def _apply_affine_to_image(img, M, t):
    """Transform ImagePlane via corner mapping (XSize/YSize + Placement)."""
    xsize, ysize = _image_dimensions(img)
    plm = img.Placement

    origin = plm.multVec(App.Vector(0, 0, 0))
    x_corner = plm.multVec(App.Vector(xsize, 0, 0))
    y_corner = plm.multVec(App.Vector(0, ysize, 0))

    origin_n = _transform_world_xy(origin, M, t)
    x_corner_n = _transform_world_xy(x_corner, M, t)
    y_corner_n = _transform_world_xy(y_corner, M, t)

    x_axis = x_corner_n - origin_n
    y_axis = y_corner_n - origin_n
    new_xsize = x_axis.Length
    new_ysize = y_axis.Length
    if new_xsize < 1e-9 or new_ysize < 1e-9:
        raise ValueError("Image size collapsed after transform")

    x_dir = App.Vector(x_axis)
    y_dir = App.Vector(y_axis)
    x_dir.normalize()
    y_dir.normalize()
    img.Placement = _placement_from_axes(origin_n, x_dir, y_dir)
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

    c0 = plm.multVec(_map_local_point(App.Vector(0, 0, 0), M, o_local))
    cx = plm.multVec(_map_local_point(App.Vector(xsize, 0, 0), M, o_local))
    cy = plm.multVec(_map_local_point(App.Vector(0, ysize, 0), M, o_local))

    x_axis = cx - c0
    y_axis = cy - c0
    new_xsize = x_axis.Length
    new_ysize = y_axis.Length
    if new_xsize < 1e-9 or new_ysize < 1e-9:
        raise ValueError("Image size collapsed after transform")

    x_dir = App.Vector(x_axis)
    y_dir = App.Vector(y_axis)
    x_dir.normalize()
    y_dir.normalize()
    img.Placement = _placement_from_axes(c0, x_dir, y_dir)
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


def apply_scale_shear(obj, origin, sx, sy, sh, mov_only=False):
    M = _symmetric_matrix(sx, sy, sh)
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
            pairs.append((obj.RefPoint, obj.MovPoint))
    return pairs


def feature_pairs_from_selection(sel):
    return [o for o in sel if image_objects.is_feature_pair(o)]


def reference_lines_from_selection(sel):
    lines = []
    for obj in sel:
        if image_objects.is_reference_line(obj) and obj.TargetLength.Value > 0:
            lines.append(obj)
    return lines


def images_from_selection(sel):
    return [o for o in sel if is_image_object(o)]


def _rms_align_error(pairs, M, t):
    errors = []
    for p_ref, p_mov in pairs:
        q = M @ np.array([p_mov.x, p_mov.y]) + t
        errors.append(np.linalg.norm(q - np.array([p_ref.x, p_ref.y])))
    return np.sqrt(np.mean(np.square(errors)))


def _pick_point(title, callback):
    view = Gui.ActiveDocument.ActiveView if Gui.ActiveDocument else None
    if view is None:
        App.Console.PrintError("Keine aktive 3D-Ansicht.\n")
        callback(None)
        return

    App.Console.PrintMessage("{} (Escape zum Abbrechen)\n".format(title))

    try:
        from pivy import coin
    except ImportError:
        App.Console.PrintError("Pivy nicht verfügbar.\n")
        callback(None)
        return

    state = {"active": True}

    def cleanup():
        if not state["active"]:
            return
        state["active"] = False
        try:
            view.removeEventCallbackPivy(
                coin.SoMouseButtonEvent.getClassTypeId(), on_click)
            view.removeEventCallbackPivy(
                coin.SoKeyboardEvent.getClassTypeId(), on_key)
        except RuntimeError:
            pass

    def on_click(event_cb):
        if not state["active"]:
            return
        event = event_cb.getEvent()
        if event.getState() != coin.SoMouseButtonEvent.DOWN:
            return
        if event.getButton() == coin.SoMouseButtonEvent.BUTTON1:
            pos = event.getPosition()
            point = view.getPoint(pos[0], pos[1])
            cleanup()
            callback(App.Vector(point))
        elif event.getButton() == coin.SoMouseButtonEvent.BUTTON2:
            cleanup()
            callback(None)

    def on_key(event_cb):
        if not state["active"]:
            return
        event = event_cb.getEvent()
        if event.getState() != coin.SoKeyboardEvent.DOWN:
            return
        if event.getKey() == coin.SoKeyboardEvent.ESCAPE:
            cleanup()
            callback(None)

    view.addEventCallbackPivy(coin.SoMouseButtonEvent.getClassTypeId(), on_click)
    view.addEventCallbackPivy(coin.SoKeyboardEvent.getClassTypeId(), on_key)


def _create_feature_pair_object(p_ref, p_mov):
    doc = App.ActiveDocument
    doc.openTransaction("Create Feature Pair")
    try:
        obj = doc.addObject("Part::FeaturePython", "FeaturePair")
        image_objects.FeaturePair(obj)
        obj.RefPoint = App.Vector(p_ref)
        obj.MovPoint = App.Vector(p_mov)
        image_objects.ViewProviderFeaturePair(obj.ViewObject)
        doc.commitTransaction()
        doc.recompute()
        App.Console.PrintMessage("Feature-Paar erstellt.\n")
    except Exception:
        doc.abortTransaction()
        raise


def create_feature_pair():
    store = {}

    def on_mov(p_mov):
        if p_mov is None:
            return
        _create_feature_pair_object(store["ref"], p_mov)

    def on_ref(p_ref):
        if p_ref is None:
            return
        store["ref"] = App.Vector(p_ref)
        _pick_point("Entsprechenden Punkt auf Bild 2 wählen", on_mov)

    _pick_point("Punkt auf Referenzbild wählen", on_ref)


class ReferenceLineDialog(object):

    def __init__(self):
        self.form = Gui.PySideUic.loadUi(
            os.path.join(os.path.dirname(__file__), "reference_line.ui"))
        self.form.setProperty("windowTitle", "Referenzlinie")
        self.form.buttonAddLine.clicked.connect(self.add_line)
        Gui.Control.showDialog(self)

    def add_line(self):
        target_length = _spinbox_value(self.form.targetLength)
        if target_length <= 0:
            App.Console.PrintError("Soll-Länge muss größer als 0 sein.\n")
            return

        store = {"target_length": target_length}

        def on_p2(p2):
            if p2 is None:
                return
            self._create_reference_line(
                store["p1"], App.Vector(p2), store["target_length"])

        def on_p1(p1):
            if p1 is None:
                return
            store["p1"] = App.Vector(p1)
            _pick_point("Linienende wählen", on_p2)

        _pick_point("Linienanfang wählen", on_p1)

    def _create_reference_line(self, p1, p2, target_length):
        doc = App.ActiveDocument
        doc.openTransaction("Create Reference Line")
        try:
            obj = doc.addObject("Part::FeaturePython", "ReferenceLine")
            image_objects.ReferenceLine(obj)
            obj.Start = p1
            obj.End = p2
            obj.TargetLength = target_length
            image_objects.ViewProviderReferenceLine(obj.ViewObject)
            doc.commitTransaction()
            doc.recompute()
            App.Console.PrintMessage(
                "Referenzlinie mit Soll-Länge {:.2f} mm erstellt.\n".format(
                    target_length))
        except Exception:
            doc.abortTransaction()
            raise

    def getStandardButtons(self):
        return _int_enum(QtGui.QDialogButtonBox.Close)

    def reject(self):
        Gui.Control.closeDialog()


def create_reference_line():
    ReferenceLineDialog()


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

        ref_corners = _snapshot_corners(img_ref_raw)

        img_ref = img_ref_raw
        img_mov = ensure_aligned_image(img_mov_raw)

        App.Console.PrintMessage(
            "Referenz: {} ({}, fix) — Bild 2: {} (wird ausgerichtet)\n".format(
                img_ref.Label,
                "AlignedImage" if image_objects.is_aligned_image(img_ref)
                else "ImagePlane",
                img_mov.Label))

        pairs = pairs_from_objects(pair_objs)
        M, t = compute_affine_2d(pairs)
        c0, c1, c3 = _aligned_corners(img_mov)
        bary_coords = [
            _barycentric_coords(pair_obj.MovPoint, c0, c1, c3)
            for pair_obj in pair_objs]
        apply_2d_transform(img_mov, M, t)
        nc0, nc1, nc3 = _aligned_corners(img_mov)
        for pair_obj, (u, v) in zip(pair_objs, bary_coords):
            pair_obj.MovPoint = _point_from_barycentric(u, v, nc0, nc1, nc3)

        if image_objects.is_aligned_image(img_ref):
            _restore_aligned_corners(img_ref, ref_corners)

        rms = _rms_align_error(pairs, M, t)
        doc.commitTransaction()
        doc.recompute()
        App.Console.PrintMessage(
            "Bilder ausgerichtet. RMS-Fehler: {:.4f} mm\n".format(rms))
    except Exception:
        doc.abortTransaction()
        raise


def solve_reference_lines():
    sel = Gui.Selection.getSelection()
    ref_lines = reference_lines_from_selection(sel)

    if len(ref_lines) < 2:
        App.Console.PrintError(
            "Mindestens 2 Referenzlinien mit Soll-Länge auswählen.\n")
        return

    doc = App.ActiveDocument
    doc.openTransaction("Solve Reference Lines")
    try:
        raw_images = _unique_images_from_selection(sel)
        pair_objs = feature_pairs_from_selection(sel)
        img_ref_raw = img_mov_raw = None
        if len(raw_images) >= 2:
            if pair_objs:
                img_ref_raw, img_mov_raw = _identify_ref_mov_images(
                    raw_images, pair_objs)
            else:
                img_ref_raw, img_mov_raw = raw_images[0], raw_images[1]

        img_ref = img_ref_raw
        ref_corners = (
            _snapshot_corners(img_ref) if img_ref is not None else None)
        img_mov = (
            ensure_aligned_image(img_mov_raw) if img_mov_raw else None)

        transform_targets = []
        if img_mov is not None:
            transform_targets.append(img_mov)
        for obj in sel:
            if image_objects.is_feature_pair(obj):
                transform_targets.append(obj)
            elif image_objects.is_reference_line(obj):
                transform_targets.append(obj)

        if not transform_targets:
            App.Console.PrintError(
                "Keine Objekte zum Transformieren in der Auswahl.\n")
            doc.abortTransaction()
            return

        sx, sy, sh = compute_scale_shear(ref_lines)
        origin = line_start_2d(ref_lines[0])
        for obj in transform_targets:
            if img_ref is not None and obj == img_ref:
                continue
            apply_scale_shear(obj, origin, sx, sy, sh, mov_only=True)

        if img_ref is not None and image_objects.is_aligned_image(img_ref):
            _restore_aligned_corners(img_ref, ref_corners)

        doc.commitTransaction()
        doc.recompute()
        App.Console.PrintMessage(
            "Kalibrierung angewendet: sx={:.6f}, sy={:.6f}, sh={:.6f}\n".format(
                sx, sy, sh))
    except Exception as exc:
        doc.abortTransaction()
        App.Console.PrintError("Kalibrierung fehlgeschlagen: {}\n".format(exc))
        raise
