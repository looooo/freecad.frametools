"""Image tools: UI, reference lines, sketch calibration."""

import math
import os
from functools import partial

import FreeCAD as App
import FreeCADGui as Gui
import numpy as np
import Part
from PySide import QtCore, QtGui

from . import image_calibration_objects
from . import image_objects
from . import image_homography as hg
from . import image_point_alignment as pa
from . import image_constraint_solver as cs


from .image_homography import compute_affine_2d, compute_homography
from .image_point_alignment import (
    apply_corner_calibration,
    apply_homography_calibration,
    apply_scale_matrix,
    apply_scale_via_homography,
    convert_selected_to_aligned_images,
    create_aligned_image_from_plane,
    ensure_aligned_image,
    feature_pairs_from_selection,
    find_aligned_image_for_source,
    images_from_selection,
    is_image_object,
    is_image_plane,
    overlay_images,
    pairs_from_objects,
    project_point_to_image,
    reference_lines_from_selection,
)
from .image_constraint_solver import (
    compute_calibration_corners,
    compute_calibration_from_specs,
)

# Backward-compatible re-exports for tests
_apply_homography_uv = hg._apply_homography_uv
_homography_from_corners = hg._homography_from_corners
_line_length_uv = hg._line_length_uv
_direction_xy_from_uv_line = hg._direction_xy_from_uv_line
_pack_corners_xy = hg._pack_corners_xy
_corner_z_values = hg._corner_z_values
_sync_warp_from_corners = pa._sync_warp_from_corners
_restore_aligned_corners = pa._restore_aligned_corners
_solve_corner_calibration = cs._solve_corner_calibration
_primary_constraint_rank = cs._primary_constraint_rank
_remap_constraint_geos = cs._remap_constraint_geos
_axis_alignment_sin = cs._axis_alignment_sin
_constraints_need_geo_remap = cs._constraints_need_geo_remap
_constraint_line_index = cs._constraint_line_index
_constraint_line_pair = cs._constraint_line_pair
_constraint_point_index = cs._constraint_point_index
_reference_image_from_selection = pa._reference_image_from_selection

_REF_LINE_ENDPOINT_SNAP_MM = cs.REF_LINE_ENDPOINT_SNAP_MM
_REF_LINE_SNAP_PIXELS = 12

def _int_enum(value):
    try:
        return int(value)
    except TypeError:
        return value
def _spinbox_value(spinbox):
    return spinbox.property("rawValue")
def _reference_lines_on_image(image, exclude=None):
    doc = image.Document if hasattr(image, "Document") else App.ActiveDocument
    if doc is None:
        return []
    out = []
    for obj in doc.Objects:
        if not image_objects.is_reference_line(obj):
            continue
        if obj == exclude:
            continue
        link = getattr(obj, "Image", None)
        if link is not None and link != image:
            continue
        out.append(obj)
    return out
def _snap_point_to_reference_endpoints(point, image, exclude_line=None, tol_mm=None):
    if tol_mm is None:
        tol_mm = _REF_LINE_ENDPOINT_SNAP_MM
    if image is None:
        return App.Vector(point)
    best = App.Vector(point)
    best_dist = tol_mm
    for line in _reference_lines_on_image(image, exclude=exclude_line):
        for candidate in (line.Start, line.End):
            dist = best.distanceToPoint(candidate)
            if dist < best_dist:
                best_dist = dist
                best = App.Vector(candidate)
    return best
def snap_reference_line_point(obj, prop):
    img = getattr(obj, "Image", None)
    if img is None:
        img = pa._reference_image_from_selection()
    if img is not None:
        pt = getattr(obj, prop)
        snapped = pa.project_point_to_image(pt, img)
        snapped = _snap_point_to_reference_endpoints(
            snapped, img, exclude_line=obj)
        if pt.distanceToPoint(snapped) > 1e-9:
            setattr(obj, prop, snapped)
        return
    pt = getattr(obj, prop)
    snapped = _snap_point_to_reference_endpoints(
        pt, img, exclude_line=obj)
    if pt.distanceToPoint(snapped) > 1e-9:
        setattr(obj, prop, snapped)


def _pick_point(title, callback, image=None):
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
            if image is not None:
                point = pa.project_point_to_image(point, image)
                point = _snap_point_to_reference_endpoints(point, image)
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
        obj = doc.addObject("App::FeaturePython", "FeaturePair")
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
class ReferenceLineEditor(object):
    """Interactive edit: drag Start/End on the image plane."""

    HANDLE_COLOR_START = (0.1, 0.75, 0.25)
    HANDLE_COLOR_END = (0.05, 0.45, 0.15)
    PICK_SCALE = 0.04

    def __init__(self, vobj):
        import FreeCADGui as Gui
        from pivy import coin

        self.vobj = vobj
        self.obj = vobj.Object
        self.view = Gui.ActiveDocument.ActiveView
        self.doc = vobj.Object.Document
        self.active = None
        self.dragging = False
        self.transaction = False
        self._closed = False

        try:
            annotation = vobj.getAnnotation()
        except AttributeError:
            annotation = vobj.Annotation
        self.root = coin.SoSeparator()
        self.start_coords = coin.SoCoordinate3()
        self.end_coords = coin.SoCoordinate3()
        self.root.addChild(self._make_handle(
            self.start_coords, self.HANDLE_COLOR_START, 14))
        self.root.addChild(self._make_handle(
            self.end_coords, self.HANDLE_COLOR_END, 14))
        annotation.addChild(self.root)
        self._update_handles()

        self.view.addEventCallbackPivy(
            coin.SoMouseButtonEvent.getClassTypeId(), self._on_mouse_button)
        self.view.addEventCallbackPivy(
            coin.SoLocation2Event.getClassTypeId(), self._on_mouse_move)
        self.view.addEventCallbackPivy(
            coin.SoKeyboardEvent.getClassTypeId(), self._on_key)
        App.Console.PrintMessage(
            "Referenzlinie bearbeiten: Endpunkte ziehen, Snap ≤ {:.0f} mm "
            "(Escape beendet).\n".format(_REF_LINE_ENDPOINT_SNAP_MM))

    def _make_handle(self, coords, color, size):
        from pivy import coin

        sep = coin.SoSeparator()
        style = coin.SoDrawStyle()
        style.pointSize = size
        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(*color)
        marker = coin.SoMarkerSet()
        marker.numPoints.setValue(1)
        sep.addChild(coords)
        sep.addChild(style)
        sep.addChild(mat)
        sep.addChild(marker)
        return sep

    def _update_handles(self):
        s = self.obj.Start
        e = self.obj.End
        self.start_coords.point.setValue(s.x, s.y, s.z)
        self.end_coords.point.setValue(e.x, e.y, e.z)

    def _pick_radius(self):
        length = self.obj.Start.distanceToPoint(self.obj.End)
        return max(8.0, length * self.PICK_SCALE)

    def _nearest_handle(self, world_point):
        p = App.Vector(world_point)
        ds = p.distanceToPoint(self.obj.Start)
        de = p.distanceToPoint(self.obj.End)
        radius = self._pick_radius()
        if ds <= radius and ds <= de:
            return "Start"
        if de <= radius:
            return "End"
        return None

    def _move_handle(self, world_point):
        img = getattr(self.obj, "Image", None)
        if img is None:
            img = _reference_image_from_selection()
        pt = pa.project_point_to_image(world_point, img) if img else App.Vector(
            world_point)
        pt = _snap_point_to_reference_endpoints(
            pt, img, exclude_line=self.obj)
        if self.active == "Start":
            self.obj.Start = pt
        elif self.active == "End":
            self.obj.End = pt
        self._update_handles()

    def _on_mouse_button(self, event_cb):
        from pivy import coin

        event = event_cb.getEvent()
        if event.getState() != coin.SoMouseButtonEvent.DOWN:
            return
        pos = event.getPosition()
        point = self.view.getPoint(pos[0], pos[1])
        handle = self._nearest_handle(point)
        if handle is None:
            return
        if event.getButton() == coin.SoMouseButtonEvent.BUTTON1:
            self.active = handle
            self.dragging = True
            if not self.transaction:
                self.doc.openTransaction("Edit Reference Line")
                self.transaction = True
            self._move_handle(point)
            self.doc.recompute()
        elif event.getButton() == coin.SoMouseButtonEvent.BUTTON2:
            self._finish()

    def _on_mouse_move(self, event_cb):
        if not self.dragging:
            return
        from pivy import coin

        event = event_cb.getEvent()
        pos = event.getPosition()
        point = self.view.getPoint(pos[0], pos[1])
        self._move_handle(point)
        self.doc.recompute()

    def _on_key(self, event_cb):
        from pivy import coin

        event = event_cb.getEvent()
        if event.getState() != coin.SoKeyboardEvent.DOWN:
            return
        if event.getKey() == coin.SoKeyboardEvent.ESCAPE:
            self._finish()

    def _finish(self):
        if self.transaction:
            self.doc.commitTransaction()
            self.transaction = False
        self.dragging = False
        self.active = None
        Gui.ActiveDocument.resetEdit()

    def close(self):
        if self._closed:
            return
        self._closed = True
        from pivy import coin

        if self.transaction:
            self.doc.abortTransaction()
            self.transaction = False
        try:
            self.view.removeEventCallbackPivy(
                coin.SoMouseButtonEvent.getClassTypeId(), self._on_mouse_button)
            self.view.removeEventCallbackPivy(
                coin.SoLocation2Event.getClassTypeId(), self._on_mouse_move)
            self.view.removeEventCallbackPivy(
                coin.SoKeyboardEvent.getClassTypeId(), self._on_key)
        except RuntimeError:
            pass
        try:
            annotation = self.vobj.getAnnotation()
        except AttributeError:
            annotation = getattr(self.vobj, "Annotation", None)
        if annotation is not None and self.root is not None:
            annotation.removeChild(self.root)
        self.root = None
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

        image = _reference_image_from_selection()
        if image is None:
            App.Console.PrintWarning(
                "Kein Bild ausgewählt — Punkte werden nicht auf "
                "eine Bildebene projiziert.\n")
        store = {"target_length": target_length, "image": image}

        def on_p2(p2):
            if p2 is None:
                return
            self._create_reference_line(
                store["p1"], App.Vector(p2), store["target_length"],
                store["image"])

        def on_p1(p1):
            if p1 is None:
                return
            store["p1"] = App.Vector(p1)
            _pick_point("Linienende wählen", on_p2, image)

        _pick_point("Linienanfang wählen", on_p1, image)

    def _create_reference_line(self, p1, p2, target_length, image=None):
        doc = App.ActiveDocument
        doc.openTransaction("Create Reference Line")
        try:
            obj = doc.addObject("Part::FeaturePython", "ReferenceLine")
            image_objects.ReferenceLine(obj)
            if image is not None:
                obj.Image = image
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
def solve_reference_lines():
    sel = Gui.Selection.getSelection()
    ref_lines = reference_lines_from_selection(sel)

    if len(ref_lines) < 1:
        App.Console.PrintError(
            "Mindestens 1 Referenzlinie mit Soll-Länge auswählen.\n")
        return

    doc = App.ActiveDocument
    doc.openTransaction("Solve Reference Lines")
    try:
        raw_images = pa._unique_images_from_selection(sel)
        pair_objs = feature_pairs_from_selection(sel)
        img_ref_raw = img_mov_raw = None
        if len(raw_images) >= 2:
            if pair_objs:
                img_ref_raw, img_mov_raw = pa._identify_ref_mov_images(
                    raw_images, pair_objs)
            else:
                img_ref_raw, img_mov_raw = raw_images[0], raw_images[1]
        elif len(raw_images) == 1:
            img_mov_raw = raw_images[0]

        img_ref = img_ref_raw
        ref_state = (
            pa._snapshot_aligned_state(img_ref)
            if img_ref is not None and image_objects.is_aligned_image(img_ref)
            else None)
        img_mov = (
            pa.ensure_aligned_image(img_mov_raw) if img_mov_raw else None)

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

        if img_mov is None or not image_objects.is_aligned_image(img_mov):
            App.Console.PrintError(
                "Scale Solver benötigt ein AlignedImage in der Auswahl.\n")
            doc.abortTransaction()
            return

        corners_new, H_new, opt_info, meta = compute_calibration_corners(
            ref_lines, img_mov)
        specs = meta.get("specs", [])
        if meta.get("endpoint_welds"):
            App.Console.PrintMessage(
                "Endpunkt-Knoten: {} Punkt(e) zusammengeführt "
                "(≤ {:.1f} mm)\n".format(
                    meta["endpoint_welds"], _REF_LINE_ENDPOINT_SNAP_MM))
        cs._print_scale_solver_debug(
            ref_lines, H_new, specs,
            "Vor Transformation", opt_info, meta)

        homography_objects = []
        for obj in transform_targets:
            if img_ref is not None and obj == img_ref:
                continue
            if pa._object_uses_image_homography(obj, img_mov):
                homography_objects.append(obj)

        pa.apply_corner_calibration(img_mov, corners_new, homography_objects)

        if img_mov is not None and image_objects.is_aligned_image(img_mov):
            pa._refresh_aligned_view(img_mov)

        if ref_state is not None:
            pa._restore_aligned_state(img_ref, ref_state)

        doc.commitTransaction()
        doc.recompute()
        cs._print_scale_solver_debug(
            ref_lines, H_new, specs,
            "Nach Transformation", meta=meta)
        E_dist = meta.get("distortion_energy", 0.0)
        rigid = meta.get("rigid_motion", {})
        exact = "exakt" if meta.get("exact") else "Ausgleich"
        App.Console.PrintMessage(
            "Kalibrierung ({} Linien, Eckpunkte, {}): "
            "E={:.4e}, Δ={:.3f} mm, θ={:.3f}°\n".format(
                len(ref_lines), exact, E_dist,
                rigid.get("translation_mm", 0.0),
                rigid.get("rotation_deg", 0.0)))
    except Exception as exc:
        doc.abortTransaction()
        App.Console.PrintError("Kalibrierung fehlgeschlagen: {}\n".format(exc))
        raise
def _sketch_line_geometries(sketch):
    out = []
    for i, geom in enumerate(sketch.Geometry):
        if isinstance(geom, Part.LineSegment):
            out.append((i, geom))
    return out
def _sketch_world_point(sketch, local_pt):
    return sketch.Placement.multVec(App.Vector(local_pt))
def _sketch_placement_from_image(img):
    c0, cx, c1, cy = pa._corners_for_image(img)
    x_axis = App.Vector(cx) - App.Vector(c0)
    y_axis = App.Vector(cy) - App.Vector(c0)
    if x_axis.Length < 1e-9 or y_axis.Length < 1e-9:
        raise ValueError("Bild-Ecken degeneriert — Sketch nicht anlegbar.")
    return hg._placement_from_axes(c0, x_axis, y_axis)
def _sketch_line_length_mm(sketch, geo_idx):
    seg = sketch.Geometry[geo_idx]
    return float(seg.StartPoint.distanceToPoint(seg.EndPoint))
def _sketch_line_label(sketch, geo_idx, seg):
    return "L{} ({:.1f} mm)".format(
        geo_idx, seg.StartPoint.distanceToPoint(seg.EndPoint))
def _weld_sketch_line_uvs(lines_meta, tol_mm=None):
    if tol_mm is None:
        tol_mm = _REF_LINE_ENDPOINT_SNAP_MM
    if not lines_meta:
        return lines_meta, 0

    entries = []
    for line in lines_meta:
        entries.append((
            (line["geo"], "start"), line["u0"], line["v0"], line["w0"]))
        entries.append((
            (line["geo"], "end"), line["u1"], line["v1"], line["w1"]))

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

    for line in lines_meta:
        line["u0"], line["v0"] = welded[(line["geo"], "start")]
        line["u1"], line["v1"] = welded[(line["geo"], "end")]
    return lines_meta, welds
def _snapshot_sketch_lines_uv(sketch, img, tol_mm=None):
    lines_meta = []
    for line_idx, (geo_idx, seg) in enumerate(_sketch_line_geometries(sketch)):
        w0 = _sketch_world_point(sketch, seg.StartPoint)
        w1 = _sketch_world_point(sketch, seg.EndPoint)
        u0, v0 = pa._uv_on_image(w0, img)
        u1, v1 = pa._uv_on_image(w1, img)
        lines_meta.append({
            "line": line_idx,
            "geo": geo_idx,
            "label": _sketch_line_label(sketch, geo_idx, seg),
            "u0": u0, "v0": v0, "u1": u1, "v1": v1,
            "w0": App.Vector(w0), "w1": App.Vector(w1),
        })
    return _weld_sketch_line_uvs(lines_meta, tol_mm)
def _store_calibration_lines(cal_obj, lines_meta):
    payload = []
    for i, line in enumerate(lines_meta):
        payload.append({
            "line": int(line.get("line", i)),
            "u0": float(line["u0"]),
            "v0": float(line["v0"]),
            "u1": float(line["u1"]),
            "v1": float(line["v1"]),
        })
    cal_obj.Lines = image_calibration_objects.dump_lines(payload)


def _point_label(point_index, pt=None):
    if pt is not None:
        label = pt.get("label")
        if label:
            return str(label)
    return "V{}".format(int(point_index))


def _iter_sketch_vertex_points_local(sketch):
    """Sketch-local points in FreeCAD wire/edge topology order."""
    shape = getattr(sketch, "Shape", None)
    if shape is None or shape.isNull():
        return
    wires = list(getattr(shape, "Wires", None) or [])
    if wires:
        for wire in wires:
            for vertex in wire.Vertexes:
                yield App.Vector(vertex.Point)
        return
    edges = list(getattr(shape, "Edges", None) or [])
    if edges:
        for edge in edges:
            for vertex in edge.Vertexes:
                yield App.Vector(vertex.Point)
        return
    for vertex in shape.Vertexes:
        yield App.Vector(vertex.Point)


def _append_point_cluster(clusters, w, u, v, tol_mm):
    w = App.Vector(w)
    for cluster in clusters:
        cw = cluster.get("w")
        if cw is not None and cw.distanceToPoint(w) <= tol_mm:
            return
    clusters.append({
        "u": float(u),
        "v": float(v),
        "w": w,
    })


def _points_meta_from_sketch(sketch, img, tol_mm=None):
    """Sketch nodes V0, V1, … in FreeCAD wire/edge vertex order."""
    if tol_mm is None:
        tol_mm = _REF_LINE_ENDPOINT_SNAP_MM
    if sketch is None or img is None:
        return []
    clusters = []
    for local_pt in _iter_sketch_vertex_points_local(sketch):
        w = _sketch_world_point(sketch, local_pt)
        u, v = pa._uv_on_image(w, img)
        _append_point_cluster(clusters, w, u, v, tol_mm)
    if not clusters:
        lines_meta, _ = _snapshot_sketch_lines_uv(sketch, img, tol_mm)
        return _points_meta_from_lines_meta(lines_meta)
    points = []
    for i, cluster in enumerate(clusters):
        points.append({
            "point": i,
            "u": cluster["u"],
            "v": cluster["v"],
            "w": cluster["w"],
            "label": _point_label(i),
        })
    return points


def _points_meta_from_lines_meta(lines_meta):
    """Fallback nodes V0, V1, … when Shape has no wire/edge vertices yet.

    Order: L0 start/end, L1 start/end, … (geometry index, StartPoint then EndPoint).
    """
    if not lines_meta:
        return []
    clusters = []
    for line in lines_meta:
        for u, v, w in (
                (line["u0"], line["v0"], line.get("w0")),
                (line["u1"], line["v1"], line.get("w1"))):
            u = float(u)
            v = float(v)
            merged = False
            for cluster in clusters:
                if (abs(cluster["u"] - u) <= 1e-8
                        and abs(cluster["v"] - v) <= 1e-8):
                    if w is not None:
                        cluster["w"] = App.Vector(w)
                    merged = True
                    break
            if not merged:
                clusters.append({
                    "u": u,
                    "v": v,
                    "w": App.Vector(w) if w is not None else None,
                })
    points = []
    for i, cluster in enumerate(clusters):
        points.append({
            "point": i,
            "u": cluster["u"],
            "v": cluster["v"],
            "w": cluster["w"],
            "label": _point_label(i),
        })
    return points


def _store_calibration_points(cal_obj, points_meta):
    image_calibration_objects.ensure_image_calibration(cal_obj)
    payload = []
    for i, pt in enumerate(points_meta):
        payload.append({
            "point": int(pt.get("point", i)),
            "u": float(pt["u"]),
            "v": float(pt["v"]),
        })
    cal_obj.Points = image_calibration_objects.dump_points(payload)


def _points_uv_differs_from_stored(fresh, stored, tol=1e-4):
    if len(fresh) != len(stored):
        return True
    for a, b in zip(fresh, stored):
        for key in ("u", "v"):
            if abs(float(a[key]) - float(b[key])) > tol:
                return True
    return False


def _calibration_image_for_points(cal_obj):
    """Image for point UV snapshots (aligned reset when available)."""
    source = _source_image_for_calibration(cal_obj)
    if not _calibration_image_is_valid(source):
        return None
    aligned = pa.find_aligned_image_for_source(source)
    if aligned is not None:
        pa._reset_aligned_from_source(aligned, source)
        return aligned
    return source


def _points_meta_for_calibration(cal_obj, sketch, img, lines_meta=None):
    """Parametric point anchors (UV) from sketch vertex order."""
    if lines_meta is None:
        lines_meta, _ = _lines_meta_for_calibration(cal_obj, sketch, img)
    fresh = _points_meta_from_sketch(sketch, img)
    if not fresh:
        fresh = _points_meta_from_lines_meta(lines_meta)
    if not fresh:
        return [], lines_meta

    stored = image_calibration_objects.parse_points(
        getattr(cal_obj, "Points", ""))
    if not stored or len(stored) != len(fresh):
        return fresh, lines_meta
    if _points_uv_differs_from_stored(fresh, stored):
        return fresh, lines_meta

    merged = []
    for i, pt in enumerate(fresh):
        s = stored[i]
        merged.append({
            "point": i,
            "u": float(s["u"]),
            "v": float(s["v"]),
            "w": pt.get("w"),
            "label": pt.get("label", _point_label(i)),
        })
    return merged, lines_meta
def _sketch_uv_differs_from_stored(fresh, stored, tol=1e-4):
    if len(fresh) != len(stored):
        return True
    for a, b in zip(fresh, stored):
        for key in ("u0", "v0", "u1", "v1"):
            if abs(float(a[key]) - float(b[key])) > tol:
                return True
    return False
def _lines_meta_for_calibration(cal_obj, sketch, img):
    """Parametric line anchors (UV) + sketch sync when geometry was edited."""
    fresh, welds = _snapshot_sketch_lines_uv(sketch, img)
    if not fresh:
        return [], 0

    stored = image_calibration_objects.parse_lines(
        getattr(cal_obj, "Lines", ""))
    if not stored or len(stored) != len(fresh):
        return fresh, welds
    if _sketch_uv_differs_from_stored(fresh, stored):
        return fresh, welds

    merged = []
    for i, line in enumerate(fresh):
        s = stored[i]
        merged.append({
            "line": i,
            "geo": line["geo"],
            "label": line.get("label", "L{}".format(i)),
            "u0": float(s["u0"]),
            "v0": float(s["v0"]),
            "u1": float(s["u1"]),
            "v1": float(s["v1"]),
            "w0": line["w0"],
            "w1": line["w1"],
        })
    merged, merge_welds = _weld_sketch_line_uvs(merged)
    return merged, welds + merge_welds
def _line_by_geo_from_snapshot(lines_meta):
    return cs._line_by_index_from_lines_meta(lines_meta)
def _rebuild_sketch_from_uv_lines(img, lines_meta, doc, label_base="CalibSketch"):
    name = doc.getUniqueObjectName(label_base.replace(" ", "_"))
    sketch = doc.addObject("Sketcher::SketchObject", name)
    _update_sketch_from_uv_lines(sketch, img, lines_meta)
    return sketch
def _update_sketch_from_uv_lines(sketch, img, lines_meta):
    """Replace sketch lines; Placement stays fixed, only geometry moves."""
    for i in range(sketch.GeometryCount - 1, -1, -1):
        sketch.delGeometry(i)
    plm_inv = sketch.Placement.inverse()
    for line in lines_meta:
        w0 = pa._world_on_image_uv(line["u0"], line["v0"], img)
        w1 = pa._world_on_image_uv(line["u1"], line["v1"], img)
        l0 = plm_inv.multVec(w0)
        l1 = plm_inv.multVec(w1)
        sketch.addGeometry(Part.LineSegment(l0, l1), False)
    return sketch
def _clone_sketch(sketch, doc, label_base="CalibSketch_aligned"):
    name = doc.getUniqueObjectName(label_base.replace(" ", "_"))
    clone = doc.addObject("Sketcher::SketchObject", name)
    clone.Label = label_base
    clone.Placement = App.Placement(sketch.Placement)
    for i in range(sketch.GeometryCount):
        clone.addGeometry(sketch.Geometry[i], False)
    return clone


def _aligned_sketch_label(original):
    label = original.Label
    if label.endswith("_aligned"):
        return label
    if label.endswith("_origin"):
        label = label[: -len("_origin")]
    return "{}_aligned".format(label)


def _linked_aligned_sketch(cal_obj):
    sketch = getattr(cal_obj, "AlignedSketch", None)
    if sketch is not None and getattr(sketch, "Document", None) is not None:
        return sketch
    sketch = getattr(cal_obj, "InputSketch", None)
    if sketch is not None and getattr(sketch, "Document", None) is not None:
        return sketch
    return None


def _delete_aligned_sketch(cal_obj):
    doc = cal_obj.Document
    if doc is None:
        return
    for prop in ("AlignedSketch", "InputSketch"):
        if prop not in cal_obj.PropertiesList:
            continue
        sketch = getattr(cal_obj, prop, None)
        if sketch is None or getattr(sketch, "Document", None) is None:
            setattr(cal_obj, prop, None)
            continue
        setattr(cal_obj, prop, None)
        doc.removeObject(sketch.Name)


def _recreate_aligned_sketch(cal_obj, img, lines_meta):
    """Fresh aligned-sketch copy from the original; replaces any previous one."""
    original = _calibration_sketch(cal_obj)
    if original is None:
        return None
    doc = cal_obj.Document
    _delete_aligned_sketch(cal_obj)
    label = _aligned_sketch_label(original)
    aligned_sk = _clone_sketch(original, doc, label_base=label)
    _update_sketch_from_uv_lines(aligned_sk, img, lines_meta)
    if "AlignedSketch" in cal_obj.PropertiesList:
        cal_obj.AlignedSketch = aligned_sk
    return aligned_sk


def _calibration_sketch(cal_obj):
    """Original user sketch (never modified by calibration solve)."""
    sketch = getattr(cal_obj, "Sketch", None)
    if sketch is not None and getattr(sketch, "Document", None) is not None:
        return sketch
    return None


def _calibration_image_is_valid(img):
    return is_image_object(img)


def _calibration_image_candidates(doc):
    if doc is None:
        return []
    return [o for o in doc.Objects if is_image_object(o)]


def _recover_calibration_image(cal_obj):
    doc = cal_obj.Document
    candidates = _calibration_image_candidates(doc)
    if not candidates:
        return None

    sketch = getattr(cal_obj, "Sketch", None)
    if sketch is not None:
        sketch_origin = sketch.Placement.Base
        best = None
        best_dist = None
        for obj in candidates:
            try:
                c0 = pa._corners_for_image(obj)[0]
                dist = c0.distanceToPoint(sketch_origin)
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best = obj
            except Exception:
                continue
        if best is not None and best_dist is not None:
            return best

    if len(candidates) == 1:
        return candidates[0]
    return None


def _source_image_for_calibration(cal_obj):
    """Original ImagePlane linked on the calibration object."""
    raw = getattr(cal_obj, "Image", None)
    img = pa._linked_object(raw)
    if img is not None and image_objects.is_aligned_image(img):
        source = getattr(img, "SourceImage", None)
        if source is not None:
            return source
    if _calibration_image_is_valid(img):
        return img
    if _calibration_image_is_valid(raw):
        return raw
    return _recover_calibration_image(cal_obj)


def _image_for_calibration(cal_obj, write_back=False):
    img = _source_image_for_calibration(cal_obj)
    if img is not None and write_back and cal_obj.Image != img:
        cal_obj.Image = img
    return img


def _line_by_geo_from_snapshot(lines_meta):
    return {int(line["geo"]): line for line in lines_meta}
def _geo_map_after_sketch_rebuild(lines_meta):
    """Legacy: map old line index -> sequential index after rebuild."""
    return {int(line.get("line", idx)): idx for idx, line in enumerate(lines_meta)}
def create_sketch_on_image(img, doc=None):
    if doc is None:
        doc = img.Document if hasattr(img, "Document") else App.ActiveDocument
    if doc is None:
        raise ValueError("Kein Dokument")
    if not _calibration_image_is_valid(img):
        raise ValueError("Bild (ImagePlane oder AlignedImage) erforderlich")
    if image_objects.is_aligned_image(img):
        pa._ensure_warp_matrix(img)
    sketch = doc.addObject(
        "Sketcher::SketchObject",
        doc.getUniqueObjectName("CalibSketch"))
    sketch.Placement = App.Placement()
    return sketch
_FRAME_WORKBENCH = "FrameWorkbench"
_calib_sketch_edit_observer = None
def _sketch_belongs_to_calibration(sketch):
    if sketch is None or sketch.TypeId != "Sketcher::SketchObject":
        return False
    doc = sketch.Document
    if doc is None:
        return False
    for obj in doc.Objects:
        if not image_calibration_objects.is_image_calibration(obj):
            continue
        if getattr(obj, "Sketch", None) == sketch:
            return True
        if getattr(obj, "AlignedSketch", None) == sketch:
            return True
        if getattr(obj, "InputSketch", None) == sketch:
            return True
    return False
class _CalibrationSketchEditObserver(object):
    """Return to Frame workbench when calibration sketch edit ends."""

    def slotResetEdit(self, vobj):
        obj = getattr(vobj, "Object", None)
        if not _sketch_belongs_to_calibration(obj):
            return
        _return_to_frame_workbench_after_sketch()
def _ensure_calibration_sketch_observer():
    global _calib_sketch_edit_observer
    if _calib_sketch_edit_observer is None:
        _calib_sketch_edit_observer = _CalibrationSketchEditObserver()
        Gui.addDocumentObserver(_calib_sketch_edit_observer)
def _return_to_frame_workbench_after_sketch():
    def _activate():
        try:
            Gui.activateWorkbench(_FRAME_WORKBENCH)
            App.Console.PrintMessage(
                "Zurück zur Frame-and-beams-Workbench.\n")
        except Exception as exc:
            App.Console.PrintWarning(
                "Frame-Workbench konnte nicht aktiviert werden: {}\n".format(
                    exc))

    QtCore.QTimer.singleShot(0, _activate)
def _activate_sketch_drawing(sketch):
    """Switch to Sketcher workbench and enter sketch edit mode."""
    if sketch is None or Gui is None:
        return

    def _enter():
        try:
            if sketch.Document is None:
                return
            _ensure_calibration_sketch_observer()
            Gui.Selection.clearSelection()
            Gui.Selection.addSelection(sketch)
            Gui.activateWorkbench("SketcherWorkbench")
            active_doc = Gui.ActiveDocument
            if active_doc is not None:
                active_doc.setEdit(sketch.Name, 0)
            App.Console.PrintMessage(
                "Sketcher: '{}' bearbeiten (Linien nachzeichnen).\n".format(
                    sketch.Label))
        except Exception as exc:
            App.Console.PrintWarning(
                "Sketcher-Modus konnte nicht gestartet werden: {}\n".format(
                    exc))

    QtCore.QTimer.singleShot(100, _enter)
def create_sketch_for_calibration(cal_obj):
    img = _image_for_calibration(cal_obj, write_back=True)
    if not _calibration_image_is_valid(img):
        App.Console.PrintError("Kein Bild verknüpft.\n")
        return
    doc = cal_obj.Document
    doc.openTransaction("Create Calibration Sketch")
    try:
        sketch = create_sketch_on_image(img, doc)
        _delete_aligned_sketch(cal_obj)
        cal_obj.Sketch = sketch
        doc.commitTransaction()
        doc.recompute()
        App.Console.PrintMessage(
            "Sketch '{}' angelegt.\n".format(sketch.Label))
        _activate_sketch_drawing(sketch)
    except Exception:
        doc.abortTransaction()
        raise
def _aligned_image_for_solve(cal_obj):
    """Working AlignedImage; reset from source plane before each solve."""
    source = _source_image_for_calibration(cal_obj)
    if not _calibration_image_is_valid(source):
        return None
    aligned = pa.find_aligned_image_for_source(source)
    if aligned is None:
        aligned = pa.create_aligned_image_from_plane(source)
    else:
        pa._reset_aligned_from_source(aligned, source)
    return aligned


def _axis_sketch_for_calibration(cal_obj):
    """Fixed sketch frame for horizontal / vertical (Sketch-X / Sketch-Y)."""
    return _calibration_sketch(cal_obj)


def solve_image_calibration(cal_obj):
    image_calibration_objects.ensure_image_calibration(cal_obj)
    source_img = _image_for_calibration(cal_obj)
    if not _calibration_image_is_valid(source_img):
        App.Console.PrintError(
            "ImageCalibration: Bild (ImagePlane oder AlignedImage) "
            "erforderlich.\n")
        return

    original_sketch = _calibration_sketch(cal_obj)
    if original_sketch is None or original_sketch.TypeId != "Sketcher::SketchObject":
        App.Console.PrintError("ImageCalibration: Sketch erforderlich.\n")
        return

    axis_sketch = _axis_sketch_for_calibration(cal_obj)

    constraints = image_calibration_objects.parse_constraints(
        cal_obj.Constraints)
    constraints = cs._remap_constraints_to_sketch(constraints, original_sketch)

    aligned = _aligned_image_for_solve(cal_obj)
    if aligned is None:
        App.Console.PrintError(
            "ImageCalibration: AlignedImage konnte nicht erzeugt werden.\n")
        return

    lines_meta, welds = _lines_meta_for_calibration(
        cal_obj, original_sketch, aligned)
    if not lines_meta:
        App.Console.PrintError("Sketch enthält keine Linien.\n")
        return

    line_by_geo = cs._line_by_index_from_lines_meta(lines_meta)
    points_meta, lines_meta = _points_meta_for_calibration(
        cal_obj, original_sketch, aligned, lines_meta=lines_meta)
    point_by_index = cs._point_by_index_from_points_meta(points_meta)
    _store_calibration_points(cal_obj, points_meta)
    length_specs = cs._length_specs_from_constraints(constraints, line_by_geo)
    if not length_specs:
        App.Console.PrintError(
            "Mindestens eine Soll-Länge in den Bedingungen definieren.\n")
        return

    if cs._has_angle_constraints(constraints):
        H0 = hg._homography_from_corners(*pa._aligned_corners(aligned))
        App.Console.PrintMessage(
            "Winkel-Bedingungen: {} horizontal, {} senkrecht, "
            "{} parallel, {} rechtwinklig\n".format(
                len(constraints.get("horizontal", [])),
                len(constraints.get("vertical", [])),
                len(constraints.get("parallel", [])),
                len(constraints.get("perpendicular", []))))
        App.Console.PrintMessage("Ausrichtung vor Optimierung:\n")
        cs._print_axis_constraint_report(
            constraints, line_by_geo, H0, axis_sketch)
        n_active = sum(
            1 for key in ("horizontal", "vertical")
            for item in constraints.get(key, [])
            if _constraint_line_index(item) in line_by_geo)
        n_declared = (
            len(constraints.get("horizontal", []))
            + len(constraints.get("vertical", [])))
        if n_declared and not n_active:
            App.Console.PrintWarning(
                "Achs-Bedingungen passen nicht zum Sketch — "
                "Bedingungen im Dialog prüfen und mit OK speichern.\n")

    doc = cal_obj.Document
    doc.openTransaction("Image Calibration Solve")
    try:
        corners_new, H_new, opt_info, meta = compute_calibration_from_specs(
            length_specs, aligned, constraints=constraints,
            line_by_geo=line_by_geo, point_by_index=point_by_index,
            sketch=axis_sketch)

        if welds:
            App.Console.PrintMessage(
                "Sketch-Endpunkte: {} zusammengeführt (≤ {:.1f} mm)\n".format(
                    welds, _REF_LINE_ENDPOINT_SNAP_MM))

        pa._restore_aligned_corners(aligned, corners_new)
        pa._sync_warp_from_corners(aligned)
        pa._refresh_aligned_view(aligned)

        aligned_sk = _recreate_aligned_sketch(cal_obj, aligned, lines_meta)
        if aligned_sk is None:
            raise RuntimeError("AlignedSketch konnte nicht erzeugt werden")

        _store_calibration_lines(cal_obj, lines_meta)
        cal_obj.Constraints = image_calibration_objects.dump_constraints(
            cs._normalize_constraints_lines(constraints, len(lines_meta)))

        doc.commitTransaction()
        doc.recompute()

        cs._print_calibration_constraint_report(
            constraints, line_by_geo, H_new, length_specs,
            sketch=axis_sketch, meta=meta, opt_info=opt_info)
        App.Console.PrintMessage(
            "Kalibrierung abgeschlossen. AlignedSketch '{}' erzeugt "
            "(aus '{}', Bild '{}').\n".format(
                aligned_sk.Label,
                original_sketch.Label,
                source_img.Label))
    except Exception as exc:
        doc.abortTransaction()
        App.Console.PrintError("Kalibrierung fehlgeschlagen: {}\n".format(exc))
        raise
def create_image_calibration():
    sel = Gui.Selection.getSelection() if Gui else []
    images = images_from_selection(sel)
    if not images:
        App.Console.PrintError("Bild (ImagePlane oder AlignedImage) auswählen.\n")
        return

    img = images[0]
    if len(images) > 1:
        aligned = [o for o in images if image_objects.is_aligned_image(o)]
        if aligned:
            img = aligned[0]
    if not _calibration_image_is_valid(img):
        App.Console.PrintError("Kein gültiges Bild ausgewählt.\n")
        return

    doc = App.ActiveDocument
    doc.openTransaction("Create Image Calibration")
    try:
        source = img
        if image_objects.is_aligned_image(img):
            plane = getattr(img, "SourceImage", None)
            if plane is not None:
                source = plane
        obj = doc.addObject("App::FeaturePython", "ImageCalibration")
        image_calibration_objects.ImageCalibration(obj)
        obj.Image = source
        sketch = create_sketch_on_image(source, doc)
        obj.Sketch = sketch
        image_calibration_objects.ViewProviderImageCalibration(obj.ViewObject)
        image_calibration_objects.ensure_image_calibration(obj)
        _ensure_calibration_sketch_observer()
        doc.commitTransaction()
        doc.recompute()
        App.Console.PrintMessage(
            "ImageCalibration erstellt — Bild '{}', Sketch '{}' "
            "(Doppelklick Sketch: Geometrie, Doppelklick Objekt: Bedingungen).\n".format(
                source.Label, sketch.Label))
    except Exception:
        doc.abortTransaction()
        raise
def draw_calibration_sketch_for(cal_obj):
    if not image_calibration_objects.is_image_calibration(cal_obj):
        App.Console.PrintError("Kein ImageCalibration-Objekt.\n")
        return
    if cal_obj.Sketch is None:
        create_sketch_for_calibration(cal_obj)
    else:
        sketch = _calibration_sketch(cal_obj)
        if sketch is not None:
            _activate_sketch_drawing(sketch)
def draw_calibration_sketch():
    sel = Gui.Selection.getSelection() if Gui else []
    cals = [
        o for o in sel
        if image_calibration_objects.is_image_calibration(o)]
    if cals:
        draw_calibration_sketch_for(cals[0])
        return
    images = images_from_selection(sel)
    if images:
        create_image_calibration()
        return
    App.Console.PrintError(
        "ImageCalibration oder Bild für neue Kalibrierung auswählen.\n")
class _CalibrationSketchLabelsOverlay(object):
    """L0, L1, … and V0, V1, … labels on sketch geometry."""

    FONT_PIXELS = 56
    OFFSET_MM = 10.0
    TEXT_SCALE = 0.55
    _active = None

    def __init__(self, sketch, points_meta=None):
        from pivy import coin

        if sketch is None or Gui is None:
            raise ValueError("Sketch erforderlich.")
        if _CalibrationSketchLabelsOverlay._active is not None:
            _CalibrationSketchLabelsOverlay._active.close()
        _CalibrationSketchLabelsOverlay._active = self

        self.sketch = sketch
        self.view = Gui.ActiveDocument.ActiveView
        self._scene_graph = self.view.getSceneGraph()
        self._closed = False
        self._entries = []
        self._scale_timer = None
        self._view_callbacks = []
        self.root = coin.SoAnnotation()
        self.labels_sep = coin.SoSeparator()
        self.root.addChild(self.labels_sep)
        self._scene_graph.addChild(self.root)

        normal = sketch.Placement.Rotation.multVec(App.Vector(0, 0, 1))
        for line_idx, (geo_idx, seg) in enumerate(
                _sketch_line_geometries(sketch)):
            w0 = _sketch_world_point(sketch, seg.StartPoint)
            w1 = _sketch_world_point(sketch, seg.EndPoint)
            mid = (App.Vector(w0) + App.Vector(w1)).multiply(0.5)
            along = App.Vector(w1) - App.Vector(w0)
            if along.Length < 1e-9:
                continue
            along.normalize()
            perp = along.cross(normal)
            if perp.Length < 1e-9:
                perp = App.Vector(0, 1, 0)
            perp.normalize()
            pos = mid + perp * self.OFFSET_MM
            self.labels_sep.addChild(
                self._make_label_node(pos, "L{}".format(line_idx)))

        for pt in points_meta or []:
            w = pt.get("w")
            if w is None:
                continue
            self.labels_sep.addChild(
                self._make_label_node(
                    App.Vector(w), _point_label(pt.get("point", 0), pt)))

        self._register_view_callbacks()
        self._start_scale_timer()
        self._update_scales()

    def _get_camera(self):
        try:
            return self.view.getCameraNode()
        except Exception:
            try:
                return self.view.getViewer().getCamera()
            except Exception:
                return None

    def _camera_is_orthographic(self, cam):
        from pivy import coin
        try:
            return cam.isOfType(coin.SoOrthographicCamera.getClassTypeId())
        except Exception:
            try:
                name = cam.getTypeId().getName().getString()
                return "Orthographic" in name
            except Exception:
                return False

    def _camera_is_perspective(self, cam):
        from pivy import coin
        try:
            return cam.isOfType(coin.SoPerspectiveCamera.getClassTypeId())
        except Exception:
            try:
                name = cam.getTypeId().getName().getString()
                return "Perspective" in name
            except Exception:
                return False

    def _vector_from_field(self, field_value):
        try:
            if hasattr(field_value, "getValue"):
                field_value = field_value.getValue()
            if hasattr(field_value, "getValue"):
                field_value = field_value.getValue()
            return App.Vector(field_value[0], field_value[1], field_value[2])
        except Exception:
            return App.Vector()

    def _register_view_callbacks(self):
        from pivy import coin

        def on_view_event(_event_cb):
            self._update_scales()

        for event_type in (
                coin.SoMouseButtonEvent.getClassTypeId(),
                coin.SoKeyboardEvent.getClassTypeId()):
            try:
                cb = self.view.addEventCallbackPivy(event_type, on_view_event)
                self._view_callbacks.append((event_type, cb))
            except Exception:
                pass

    def _start_scale_timer(self):
        self._scale_timer = QtCore.QTimer()
        self._scale_timer.timeout.connect(self._update_scales)
        self._scale_timer.start(200)

    def _viewport_height(self):
        try:
            size = self.view.getSize()
            if size and len(size) >= 2:
                return max(float(size[1]), 1.0)
        except Exception:
            pass
        return 480.0

    def _world_per_pixel(self, position):
        cam = self._get_camera()
        if cam is None:
            return 1.0 / self._viewport_height()
        vh = self._viewport_height()
        if self._camera_is_orthographic(cam):
            try:
                return float(cam.height.getValue()) / vh
            except Exception:
                return 1.0 / vh
        if self._camera_is_perspective(cam):
            try:
                cam_pos = self._vector_from_field(cam.position.getValue())
                dist = cam_pos.sub(position).Length
                if dist < 1e-9:
                    dist = 1.0
                fov = float(cam.fieldOfView.getValue())
                return 2.0 * dist * math.tan(fov * 0.5) / vh
            except Exception:
                return 1.0 / vh
        return 1.0 / vh

    def _make_label_node(self, position, text):
        from pivy import coin

        sep = coin.SoSeparator()
        depth = coin.SoDepthBuffer()
        depth.test = False
        sep.addChild(depth)
        trans = coin.SoTransform()
        trans.translation.setValue(position.x, position.y, position.z)
        sep.addChild(trans)
        scale = coin.SoScale()
        sep.addChild(scale)
        self._entries.append((App.Vector(position), scale))
        light = coin.SoLightModel()
        light.model = coin.SoLightModel.BASE_COLOR
        sep.addChild(light)
        font = coin.SoFont()
        font.size = 1.0
        sep.addChild(font)
        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(0.0, 0.0, 0.0)
        mat.emissiveColor.setValue(0.0, 0.0, 0.0)
        sep.addChild(mat)
        text3 = coin.SoText3()
        text3.string = text
        text3.justification = coin.SoText3.CENTER
        sep.addChild(text3)
        return sep

    def _update_scales(self, *args):
        if self._closed or self.root is None:
            return
        for position, scale in self._entries:
            factor = (
                self._world_per_pixel(position)
                * self.FONT_PIXELS
                * self.TEXT_SCALE)
            scale.scaleFactor.setValue(factor, factor, factor)

    @classmethod
    def close_active(cls):
        active = cls._active
        if active is not None:
            active.close()

    def close(self):
        if self._closed:
            return
        self._closed = True
        if _CalibrationSketchLabelsOverlay._active is self:
            _CalibrationSketchLabelsOverlay._active = None
        if self._scale_timer is not None:
            try:
                self._scale_timer.stop()
                self._scale_timer.timeout.disconnect(self._update_scales)
            except Exception:
                pass
            self._scale_timer = None
        view = getattr(self, "view", None)
        for event_type, cb in self._view_callbacks:
            try:
                if view is not None:
                    view.removeEventCallbackPivy(event_type, cb)
            except Exception:
                pass
        self._view_callbacks = []
        self._entries = []
        root = self.root
        scene_graph = getattr(self, "_scene_graph", None)
        view = getattr(self, "view", None)
        self.root = None
        self.labels_sep = None
        self._scene_graph = None
        if root is not None:
            removed = False
            if scene_graph is not None:
                try:
                    scene_graph.removeChild(root)
                    removed = True
                except Exception:
                    pass
            if not removed:
                try:
                    parent = root.getParent()
                    if parent is not None:
                        parent.removeChild(root)
                except Exception:
                    pass
        if view is not None:
            try:
                view.requestRedraw()
            except Exception:
                pass
_active_constraints_dialog = None
class ImageCalibrationConstraintsDialog(object):

    def __init__(self, cal_obj):
        global _active_constraints_dialog
        if _active_constraints_dialog is not None:
            _active_constraints_dialog._close_overlay()
        _active_constraints_dialog = self

        self.cal_obj = cal_obj
        self._building = False
        self._label_overlay = None
        self._refresh_sketch_lines()
        self._refresh_sketch_points()
        raw_constraints = image_calibration_objects.parse_constraints(
            cal_obj.Constraints)
        self.constraints = cs._remap_constraints_to_sketch(
            raw_constraints, self.sketch)
        if _constraints_need_geo_remap(raw_constraints, self.sketch):
            cal_obj.Constraints = image_calibration_objects.dump_constraints(
                self.constraints)

        self.form = QtGui.QWidget()
        self.form.setWindowTitle("Kalibrierungs-Bedingungen")
        self.form.destroyed.connect(self._on_form_destroyed)
        layout = QtGui.QVBoxLayout(self.form)

        hint = QtGui.QLabel(
            "Parametrisch: Kanten L0, L1, … und Knoten V0, V1, … (FreeCAD-"
            "Sketch-Vertex-Reihenfolge) am Objekt gespeichert. Minimale "
            "Bedingungen setzen, „Kalibrieren“, Ergebnis prüfen, weitere "
            "Bedingungen ergänzen, erneut kalibrieren.\n"
            "Horizontal / Senkrecht: Kante parallel zur festen Sketch-Achse "
            "(Sketch-X bzw. Sketch-Y) — nur das Bild wird verzerrt, die "
            "Sketch-Platzierung bleibt unverändert; nach dem Solve werden "
            "nur die Linien-Endpunkte neu gesetzt.\n"
            "Fixer Punkt: Knoten Vk soll an Soll-X/Y (Welt-mm) liegen. "
            "Bei gesetzten Fixpunkten entfällt die Schwerpunkt-Nebenbedingung.")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        self.table = QtGui.QTableWidget(0, 4)
        self.table.horizontalHeader().hide()
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(
            QtGui.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(
            QtGui.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(
            QtGui.QAbstractItemView.NoEditTriggers)
        self.table.setColumnWidth(0, 100)
        self.table.setColumnWidth(1, 140)
        self.table.setColumnWidth(2, 120)
        layout.addWidget(self.table)

        btn_row = QtGui.QHBoxLayout()
        for label, handler in (
                ("Soll-Länge", self._add_length_row),
                ("Parallel", self._add_parallel_row),
                ("Rechtwinklig", self._add_rechtwinklig_row),
                ("Horizontal", self._add_horizontal_row),
                ("Senkrecht", self._add_senkrecht_row),
                ("Fixer Punkt", self._add_fixed_point_row)):
            btn = QtGui.QPushButton(label)
            btn.clicked.connect(handler)
            btn_row.addWidget(btn)
        btn_del = QtGui.QPushButton("Entfernen")
        btn_del.clicked.connect(self._remove_row)
        btn_row.addWidget(btn_del)
        btn_solve = QtGui.QPushButton("Kalibrieren")
        btn_solve.clicked.connect(self._solve_and_refresh)
        btn_row.addWidget(btn_solve)
        layout.addLayout(btn_row)

        self._load_constraints()
        if self.sketch is not None:
            try:
                self._label_overlay = _CalibrationSketchLabelsOverlay(
                    self.sketch, self.points)
            except Exception as exc:
                App.Console.PrintWarning(
                    "Sketch-Beschriftung fehlgeschlagen: {}\n".format(exc))
        Gui.Control.showDialog(self)

    def isModal(self):
        return False

    def getStandardButtons(self):
        return _int_enum(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)

    def _refresh_sketch_lines(self):
        self.sketch = _calibration_sketch(self.cal_obj)
        self.lines = []
        if self.sketch is not None:
            for line_idx, (geo_idx, seg) in enumerate(
                    _sketch_line_geometries(self.sketch)):
                self.lines.append({
                    "line": line_idx,
                    "geo": geo_idx,
                    "line_idx": line_idx,
                    "label": "L{} ({:.1f} mm)".format(
                        line_idx,
                        seg.StartPoint.distanceToPoint(seg.EndPoint)),
                })

    def _refresh_sketch_points(self):
        self.points = []
        if self.sketch is None:
            return
        img = _calibration_image_for_points(self.cal_obj)
        if not _calibration_image_is_valid(img):
            return
        self.points = _points_meta_from_sketch(self.sketch, img)

    def _on_form_destroyed(self, *args):
        global _active_constraints_dialog
        if _active_constraints_dialog is not self:
            return
        self._close_overlay()

    def _close_overlay(self):
        global _active_constraints_dialog
        overlay = getattr(self, "_label_overlay", None)
        if overlay is not None:
            overlay.close()
            self._label_overlay = None
        if _active_constraints_dialog is self:
            _active_constraints_dialog = None

    def clicked(self, button):
        std_ok = _int_enum(QtGui.QDialogButtonBox.Ok)
        std_cancel = _int_enum(QtGui.QDialogButtonBox.Cancel)
        if button == std_ok:
            self.accept()
        elif button == std_cancel:
            self.reject()
        else:
            self._close_overlay()
            Gui.Control.closeDialog()

    def _geo_combo(self, line_idx=None):
        combo = QtGui.QComboBox(self.table)
        role = QtCore.Qt.UserRole
        for line in self.lines:
            combo.addItem(line["label"])
            combo.setItemData(
                combo.count() - 1, int(line["line"]), role)
        if line_idx is not None:
            line_idx = int(line_idx)
            idx = combo.findData(line_idx, role)
            if idx < 0:
                for i in range(combo.count()):
                    if int(combo.itemData(i, role)) == line_idx:
                        idx = i
                        break
            if idx >= 0:
                combo.setCurrentIndex(idx)
            elif combo.count() > 0:
                combo.setCurrentIndex(0)
        combo.setMinimumContentsLength(14)
        try:
            combo.setSizeAdjustPolicy(
                QtGui.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        except Exception:
            pass
        return combo

    def _combo_geo(self, combo):
        if combo is None:
            return None
        idx = combo.currentIndex()
        if idx < 0:
            return None
        data = combo.itemData(idx, QtCore.Qt.UserRole)
        if data is None:
            return None
        return int(data)

    def _point_combo(self, point_idx=None):
        combo = QtGui.QComboBox(self.table)
        role = QtCore.Qt.UserRole
        for pt in self.points:
            combo.addItem(_point_label(pt["point"], pt))
            combo.setItemData(
                combo.count() - 1, int(pt["point"]), role)
        if point_idx is not None:
            point_idx = int(point_idx)
            idx = combo.findData(point_idx, role)
            if idx < 0:
                for i in range(combo.count()):
                    if int(combo.itemData(i, role)) == point_idx:
                        idx = i
                        break
            if idx >= 0:
                combo.setCurrentIndex(idx)
            elif combo.count() > 0:
                combo.setCurrentIndex(0)
        combo.setMinimumContentsLength(10)
        try:
            combo.setSizeAdjustPolicy(
                QtGui.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        except Exception:
            pass
        return combo

    def _combo_point(self, combo):
        if combo is None:
            return None
        idx = combo.currentIndex()
        if idx < 0:
            return None
        data = combo.itemData(idx, QtCore.Qt.UserRole)
        if data is None:
            return None
        return int(data)

    def _world_xy_for_point(self, point_idx):
        for pt in self.points:
            if int(pt.get("point", -1)) == int(point_idx):
                w = pt.get("w")
                if w is not None:
                    return float(w.x), float(w.y)
        return 0.0, 0.0

    def _length_spin(self, value=None):
        spin = QtGui.QDoubleSpinBox()
        spin.setRange(0.0, 1e9)
        spin.setDecimals(3)
        spin.setSuffix(" mm")
        if value is not None:
            spin.setValue(value)
        return spin

    def _spin_value(self, spin):
        return float(spin.value())

    def _coord_spin(self, value=None):
        spin = QtGui.QDoubleSpinBox()
        spin.setRange(-1e9, 1e9)
        spin.setDecimals(3)
        spin.setSuffix(" mm")
        if value is not None:
            spin.setValue(value)
        return spin

    def _set_dash_cell(self, row, col):
        self.table.setItem(row, col, QtGui.QTableWidgetItem("-"))

    def _load_constraints(self):
        self._building = True
        for item in self.constraints.get("lengths", []):
            self._add_row(
                "Soll-Länge",
                line_a=_constraint_line_index(item),
                target_mm=float(item["target_mm"]))
        for item in self.constraints.get("parallel", []):
            la, lb = _constraint_line_pair(item)
            self._add_row("Parallel", line_a=la, line_b=lb)
        for item in self.constraints.get("perpendicular", []):
            la, lb = _constraint_line_pair(item)
            self._add_row("Rechtwinklig", line_a=la, line_b=lb)
        for item in self.constraints.get("horizontal", []):
            self._add_row(
                "Horizontal", line_a=_constraint_line_index(item))
        for item in self.constraints.get("vertical", []):
            self._add_row(
                "Senkrecht", line_a=_constraint_line_index(item))
        for item in self.constraints.get("fixed_points", []):
            self._add_row(
                "Fixer Punkt",
                point_idx=_constraint_point_index(item),
                x_mm=float(item["target_x_mm"]),
                y_mm=float(item["target_y_mm"]))
        self._building = False

    _SINGLE_EDGE_KINDS = ("Horizontal", "Senkrecht")

    def _add_row(self, kind, line_a=None, geo_a=None, line_b=None, geo_b=None,
                 target_mm=None, point_idx=None, x_mm=None, y_mm=None):
        if line_a is None:
            line_a = geo_a
        if line_b is None:
            line_b = geo_b
        if kind == "Fixer Punkt":
            if point_idx is None and not self._building and self.points:
                point_idx = self.points[0]["point"]
            if x_mm is None and y_mm is None and point_idx is not None:
                x_mm, y_mm = self._world_xy_for_point(point_idx)
        elif line_a is None and not self._building and self.lines:
            line_a = self.lines[0]["line"]
        if kind == "Soll-Länge":
            if target_mm is None and self.sketch is not None and line_a is not None:
                for line in self.lines:
                    if line["line"] == line_a:
                        target_mm = _sketch_line_length_mm(
                            self.sketch, line["geo"])
                        break
        elif kind in ("Parallel", "Rechtwinklig"):
            if line_b is None and not self._building and self.lines:
                line_b = (
                    self.lines[1]["line"] if len(self.lines) > 1 else line_a)

        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QtGui.QTableWidgetItem(kind))
        if kind == "Fixer Punkt":
            self.table.setCellWidget(row, 1, self._point_combo(point_idx))
            self.table.setCellWidget(row, 2, self._coord_spin(x_mm))
            self.table.setCellWidget(row, 3, self._coord_spin(y_mm))
        else:
            self.table.setCellWidget(row, 1, self._geo_combo(line_a))
            if kind == "Soll-Länge":
                self.table.setCellWidget(row, 2, self._length_spin(target_mm))
                self._set_dash_cell(row, 3)
            elif kind in self._SINGLE_EDGE_KINDS:
                self._set_dash_cell(row, 2)
                self._set_dash_cell(row, 3)
            else:
                self.table.setCellWidget(row, 2, self._geo_combo(line_b))
                self._set_dash_cell(row, 3)

    def _add_length_row(self, geo=None, target_mm=None):
        self._add_row("Soll-Länge", line_a=geo, target_mm=target_mm)

    def _add_parallel_row(self, geo_a=None, geo_b=None):
        self._add_row("Parallel", line_a=geo_a, line_b=geo_b)

    def _add_rechtwinklig_row(self, geo_a=None, geo_b=None):
        self._add_row("Rechtwinklig", line_a=geo_a, line_b=geo_b)

    def _add_senkrecht_row(self, geo=None):
        self._add_row("Senkrecht", line_a=geo)

    def _add_horizontal_row(self, geo=None):
        self._add_row("Horizontal", line_a=geo)

    def _add_fixed_point_row(self, point_idx=None, x_mm=None, y_mm=None):
        if not self.points:
            App.Console.PrintMessage(
                "Keine Sketch-Knoten — zuerst Linien im Sketch zeichnen.\n")
            return
        self._add_row(
            "Fixer Punkt", point_idx=point_idx, x_mm=x_mm, y_mm=y_mm)

    def _remove_row(self):
        row = self.table.currentRow()
        if row < 0:
            App.Console.PrintMessage("Zeile in der Tabelle auswählen.\n")
            return
        self.table.removeRow(row)

    def _collect_constraints(self):
        constraints = image_calibration_objects.default_constraints()
        for row in range(self.table.rowCount()):
            kind_item = self.table.item(row, 0)
            if kind_item is None:
                continue
            kind = kind_item.text()
            if kind == "Fixer Punkt":
                combo_pt = self.table.cellWidget(row, 1)
                pi = self._combo_point(combo_pt)
                if pi is None:
                    App.Console.PrintWarning(
                        "Fixpunkt-Zeile übersprungen: ungültiger Knoten.\n")
                    continue
                spin_x = self.table.cellWidget(row, 2)
                spin_y = self.table.cellWidget(row, 3)
                if spin_x is None or spin_y is None:
                    continue
                constraints["fixed_points"].append({
                    "point": pi,
                    "target_x_mm": self._spin_value(spin_x),
                    "target_y_mm": self._spin_value(spin_y),
                })
                continue
            combo_a = self.table.cellWidget(row, 1)
            la = self._combo_geo(combo_a)
            if la is None:
                continue
            if kind == "Soll-Länge":
                spin = self.table.cellWidget(row, 2)
                if spin is None:
                    continue
                constraints["lengths"].append({
                    "line": la,
                    "target_mm": self._spin_value(spin),
                })
            elif kind == "Horizontal":
                constraints["horizontal"].append({"line": la})
            elif kind == "Senkrecht":
                constraints["vertical"].append({"line": la})
            elif kind in ("Parallel", "Rechtwinklig"):
                combo_b = self.table.cellWidget(row, 2)
                lb = self._combo_geo(combo_b)
                if lb is None:
                    continue
                if kind == "Parallel":
                    constraints["parallel"].append(
                        {"line_a": la, "line_b": lb})
                else:
                    constraints["perpendicular"].append(
                        {"line_a": la, "line_b": lb})
        return constraints

    def _save_constraints(self, constraints):
        self.cal_obj.Constraints = image_calibration_objects.dump_constraints(
            constraints)
        self.constraints = constraints

    def _sync_lines_from_sketch(self):
        source = _source_image_for_calibration(self.cal_obj)
        if source is None:
            return
        aligned = pa.find_aligned_image_for_source(source)
        if aligned is not None:
            pa._reset_aligned_from_source(aligned, source)
        else:
            aligned = source
        sketch = _calibration_sketch(self.cal_obj)
        if sketch is None:
            return
        lines_meta, _ = _snapshot_sketch_lines_uv(sketch, aligned)
        if lines_meta:
            _store_calibration_lines(self.cal_obj, lines_meta)
            points_meta = _points_meta_from_sketch(sketch, aligned)
            if not points_meta:
                points_meta = _points_meta_from_lines_meta(lines_meta)
            _store_calibration_points(self.cal_obj, points_meta)

    def _refresh_after_solve(self):
        self._refresh_sketch_lines()
        self._refresh_sketch_points()
        self.constraints = image_calibration_objects.parse_constraints(
            self.cal_obj.Constraints)
        self.table.setRowCount(0)
        self._building = True
        self._load_constraints()
        self._building = False
        if self.sketch is not None:
            try:
                self._label_overlay = _CalibrationSketchLabelsOverlay(
                    self.sketch, self.points)
            except Exception as exc:
                App.Console.PrintWarning(
                    "Sketch-Beschriftung fehlgeschlagen: {}\n".format(exc))

    def _solve_and_refresh(self):
        constraints = self._collect_constraints()
        if not constraints.get("lengths"):
            App.Console.PrintError(
                "Mindestens eine Soll-Länge in den Bedingungen definieren.\n")
            return
        self._save_constraints(constraints)
        self._sync_lines_from_sketch()
        self._close_overlay()
        try:
            solve_image_calibration(self.cal_obj)
        except Exception:
            self._refresh_after_solve()
            raise
        self._refresh_after_solve()

    def accept(self):
        try:
            constraints = self._collect_constraints()
            self._save_constraints(constraints)
            self._sync_lines_from_sketch()
        finally:
            self._close_overlay()
        Gui.Control.closeDialog()
        return True

    def reject(self):
        self._close_overlay()
        Gui.Control.closeDialog()
        return True
def show_calibration_constraints_dialog(cal_obj):
    if not image_calibration_objects.is_image_calibration(cal_obj):
        App.Console.PrintError("Kein ImageCalibration-Objekt.\n")
        return
    image_calibration_objects.ensure_image_calibration(cal_obj)
    ImageCalibrationConstraintsDialog(cal_obj)
def show_calibration_constraints_for_selection():
    sel = Gui.Selection.getSelection() if Gui else []
    cals = [
        o for o in sel
        if image_calibration_objects.is_image_calibration(o)]
    if not cals:
        App.Console.PrintError("ImageCalibration-Objekt auswählen.\n")
        return
    show_calibration_constraints_dialog(cals[0])
_REF_LINE_ENDPOINT_SNAP_MM = 5.0
