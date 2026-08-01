import os

import FreeCAD as App
import Part

from freecad.frametools import ICON_PATH

REF_COLOR = (0.17, 0.47, 0.93)
MOV_COLOR = (0.93, 0.27, 0.17)
LINE_COLOR = (0.4, 0.4, 0.4)
MARKER_SIZE = 9
_SWITCH_ON = 0
_SWITCH_OFF = -1


def reference_line_length_xy(obj):
    dx = obj.End.x - obj.Start.x
    dy = obj.End.y - obj.Start.y
    return (dx * dx + dy * dy) ** 0.5


def reference_line_length_3d(obj):
    return obj.Start.distanceToPoint(obj.End)


class FeaturePair(object):

    def __init__(self, obj):
        obj.addProperty(
            "App::PropertyVector", "RefPoint", "Feature",
            "Punkt auf Referenzbild").RefPoint = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "MovPoint", "Feature",
            "Entsprechender Punkt auf Bild 2").MovPoint = App.Vector()
        obj.addProperty(
            "App::PropertyBool", "ShowMarkers", "Feature",
            "Marker und Verbindungslinie anzeigen").ShowMarkers = True
        obj.Proxy = self

    def onChanged(self, obj, prop):
        if prop != "ShowMarkers":
            return
        vobj = getattr(obj, "ViewObject", None)
        if vobj is None:
            return
        vobj.Visibility = bool(obj.ShowMarkers)

    def execute(self, obj):
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ViewProviderFeaturePair(object):

    def __init__(self, vobj):
        vobj.Proxy = self
        self.attach(vobj)

    def attach(self, vobj):
        import FreeCADGui as Gui
        from pivy import coin

        self.ViewObject = vobj
        self.Object = vobj.Object
        _ensure_show_markers(self.Object)
        self._clear_legacy_annotation(vobj)

        if getattr(self, "_scene_attached", False):
            self._apply_visibility(vobj)
            self._update_markers()
            return

        self.switch = coin.SoSwitch()
        self.root = coin.SoSeparator()

        self.ref_sep, self.ref_coords = self._make_marker(Gui, REF_COLOR)
        self.mov_sep, self.mov_coords = self._make_marker(Gui, MOV_COLOR)
        self.root.addChild(self.ref_sep)
        self.root.addChild(self.mov_sep)

        self.line_coords = coin.SoCoordinate3()
        line_style = coin.SoDrawStyle()
        line_style.lineWidth = 2
        line_mat = coin.SoMaterial()
        line_mat.diffuseColor.setValue(*LINE_COLOR)
        self.line_set = coin.SoLineSet()
        self.line_set.numVertices.setValue(2)
        self.root.addChild(self.line_coords)
        self.root.addChild(line_style)
        self.root.addChild(line_mat)
        self.root.addChild(self.line_set)

        self.switch.addChild(self.root)
        self._use_annotation = vobj.Object.TypeId != "App::FeaturePython"
        if self._use_annotation:
            self._add_switch_to_annotation(vobj)
        else:
            vobj.addDisplayMode(self.switch, "Feature")
            modes = vobj.listDisplayModes()
            if "Feature" in modes:
                vobj.DisplayMode = "Feature"
        self._update_markers()
        self._apply_visibility(vobj)
        self._scene_attached = True

    def _add_switch_to_annotation(self, vobj):
        try:
            annotation = vobj.getAnnotation()
        except AttributeError:
            annotation = getattr(vobj, "Annotation", None)
        if annotation is not None:
            annotation.addChild(self.switch)

    def _clear_legacy_annotation(self, vobj):
        try:
            annotation = vobj.getAnnotation()
        except AttributeError:
            annotation = getattr(vobj, "Annotation", None)
        if annotation is None:
            return
        while annotation.getNumChildren():
            annotation.removeChild(0)

    def _apply_visibility(self, vobj):
        if not hasattr(self, "switch"):
            return
        if vobj.Visibility:
            self.switch.whichChild = _SWITCH_ON
        else:
            self.switch.whichChild = _SWITCH_OFF

    def _make_marker(self, gui, color):
        from pivy import coin

        sep = coin.SoSeparator()
        coords = coin.SoCoordinate3()
        style = coin.SoDrawStyle()
        style.pointSize = 12
        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(*color)
        marker = coin.SoMarkerSet()
        marker.markerIndex = gui.getMarkerIndex("CIRCLE_FILLED", MARKER_SIZE)
        sep.addChild(coords)
        sep.addChild(style)
        sep.addChild(mat)
        sep.addChild(marker)
        return sep, coords

    def _update_markers(self):
        if not hasattr(self, "ref_coords"):
            return
        ref = self.Object.RefPoint
        mov = self.Object.MovPoint
        self.ref_coords.point.setValue(ref.x, ref.y, ref.z)
        self.mov_coords.point.setValue(mov.x, mov.y, mov.z)
        self.line_coords.point.setValues([
            (ref.x, ref.y, ref.z),
            (mov.x, mov.y, mov.z),
        ])

    def updateData(self, obj, prop):
        if isinstance(prop, str):
            props = [prop]
        else:
            props = list(prop)
        if not props or any(p in props for p in ("RefPoint", "MovPoint")):
            self._update_markers()

    def onChanged(self, vobj, prop):
        if prop == "Visibility":
            self._apply_visibility(vobj)
            obj = vobj.Object
            if hasattr(obj, "ShowMarkers") and obj.ShowMarkers != vobj.Visibility:
                obj.ShowMarkers = vobj.Visibility

    def getDisplayModes(self, obj):
        if getattr(obj, "TypeId", "") == "App::FeaturePython":
            return ["Feature"]
        return []

    def getDefaultDisplayMode(self):
        obj = getattr(self, "Object", None)
        if obj and obj.TypeId == "App::FeaturePython":
            return "Feature"
        return "Wireframe"

    def setDisplayMode(self, mode):
        return mode

    def getIcon(self):
        return os.path.join(ICON_PATH, "feature_pair.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        if getattr(self, "ViewObject", None):
            self.attach(self.ViewObject)


class ReferenceLine(object):

    def __init__(self, obj):
        obj.addProperty(
            "App::PropertyLink", "Image", "Line",
            "Bildebene für Start/Ende")
        obj.addProperty(
            "App::PropertyVector", "Start", "Line", "Linienanfang").Start = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "End", "Line", "Linienende").End = App.Vector()
        obj.addProperty(
            "App::PropertyLength", "TargetLength", "Line",
            "Soll-Länge").TargetLength = 0
        obj.addProperty(
            "App::PropertyLength", "CurrentLength", "Line",
            "Aktuelle Länge")
        obj.CurrentLength = 0
        obj.setEditorMode("CurrentLength", 1)
        obj.Proxy = self

    def _snap_to_image(self, obj, prop):
        from freecad.frametools import image_tools
        image_tools.snap_reference_line_point(obj, prop)

    def _update_current_length(self, obj):
        _ensure_current_length(obj)
        length = reference_line_length_xy(obj)
        if abs(obj.CurrentLength.Value - length) > 1e-9:
            obj.CurrentLength = length

    def onChanged(self, obj, prop):
        if prop in ("Start", "End"):
            self._snap_to_image(obj, prop)
            self._update_current_length(obj)

    def execute(self, obj):
        self._update_current_length(obj)
        if obj.Start.distanceToPoint(obj.End) < 1e-9:
            return
        obj.Shape = Part.makeLine(obj.Start, obj.End)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def _ensure_current_length(obj):
    if hasattr(obj, "CurrentLength"):
        return
    obj.addProperty(
        "App::PropertyLength", "CurrentLength", "Line",
        "Aktuelle Länge")
    obj.CurrentLength = reference_line_length_xy(obj)
    obj.setEditorMode("CurrentLength", 1)


class ViewProviderReferenceLine(object):

    def __init__(self, vobj):
        vobj.Proxy = self
        self.editor = None
        self.attach(vobj)

    def attach(self, vobj):
        self.ViewObject = vobj
        self.Object = vobj.Object
        if not hasattr(self, "editor"):
            self.editor = None
        ensure_reference_line(vobj.Object)
        vobj.LineColor = (0.1, 0.6, 0.2)
        vobj.LineWidth = 3

    def updateData(self, obj, prop):
        if isinstance(prop, str):
            props = [prop]
        else:
            props = list(prop)
        editor = getattr(self, "editor", None)
        if editor and any(p in props for p in ("Start", "End")):
            editor._update_handles()

    def doubleClicked(self, vobj):
        import FreeCADGui as Gui
        if vobj.Document:
            Gui.ActiveDocument.setEdit(vobj.ObjectName, 0)
            return True
        return False

    def setEdit(self, vobj, mode):
        from freecad.frametools import image_tools
        editor = getattr(self, "editor", None)
        if editor is not None:
            editor.close()
        self.editor = image_tools.ReferenceLineEditor(vobj)
        return True

    def unsetEdit(self, vobj):
        editor = getattr(self, "editor", None)
        if editor is not None:
            editor.close()
        self.editor = None
        return True

    def getIcon(self):
        return os.path.join(ICON_PATH, "reference_line.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        self.editor = None
        if getattr(self, "ViewObject", None):
            self.attach(self.ViewObject)


def is_feature_pair(obj):
    return getattr(getattr(obj, "Proxy", None), "__class__", None).__name__ == "FeaturePair"


def _ensure_show_markers(obj):
    if not hasattr(obj, "ShowMarkers"):
        obj.addProperty(
            "App::PropertyBool", "ShowMarkers", "Feature",
            "Marker und Verbindungslinie anzeigen")
        obj.ShowMarkers = True


def ensure_feature_pair(obj):
    if not is_feature_pair(obj):
        return
    _ensure_show_markers(obj)
    if hasattr(obj, "Shape") and not obj.Shape.isNull():
        obj.Shape = Part.Shape()
    vobj = getattr(obj, "ViewObject", None)
    if vobj and getattr(vobj, "Proxy", None):
        vobj.Proxy.attach(vobj)


def is_reference_line(obj):
    return getattr(getattr(obj, "Proxy", None), "__class__", None).__name__ == "ReferenceLine"


def ensure_reference_line(obj):
    if not is_reference_line(obj):
        return
    if not hasattr(obj, "Image"):
        obj.addProperty(
            "App::PropertyLink", "Image", "Line",
            "Bildebene für Start/Ende")
    if not obj.Image:
        from freecad.frametools import image_tools
        guess = image_tools._guess_image_for_line(obj)
        if guess is not None:
            obj.Image = guess
    _ensure_current_length(obj)
    obj.setEditorMode("CurrentLength", 1)
    if getattr(obj, "Proxy", None):
        obj.Proxy._update_current_length(obj)
        for prop in ("Start", "End"):
            obj.Proxy._snap_to_image(obj, prop)


class AlignedImage(object):

    def __init__(self, obj):
        obj.addProperty(
            "App::PropertyFile", "ImageFile", "Image",
            "Pfad zur Bilddatei").ImageFile = ""
        obj.addProperty(
            "App::PropertyVector", "Corner0", "Image",
            "Ursprung (UV 0,0)").Corner0 = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "CornerX", "Image",
            "Ecke entlang U (UV 1,0)").CornerX = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "CornerY", "Image",
            "Ecke entlang V (UV 0,1)").CornerY = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "Corner1", "Image",
            "Ecke diagonal (UV 1,1)").Corner1 = App.Vector()
        obj.addProperty(
            "App::PropertyMatrix", "WarpMatrix", "Image",
            "Projektive Abbildung UV-Einheitsquad -> Welt").WarpMatrix = App.Matrix()
        obj.addProperty(
            "App::PropertyLink", "SourceImage", "Image",
            "Ursprüngliches ImagePlane-Objekt")
        obj.Proxy = self

    def onChanged(self, obj, prop):
        if prop not in (
                "WarpMatrix", "ImageFile",
                "Corner0", "CornerX", "CornerY", "Corner1"):
            return
        vobj = getattr(obj, "ViewObject", None)
        if vobj and getattr(vobj, "Proxy", None):
            vobj.Proxy.updateData(obj, prop)

    def onDocumentRestored(self, obj):
        from freecad.frametools import image_point_alignment as pa

        pa.ensure_aligned_image_file(obj)
        pa._ensure_corner1_property(obj)
        pa._ensure_warp_matrix(obj)

    def execute(self, obj):
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ViewProviderAlignedImage(object):

    def __init__(self, vobj):
        vobj.Proxy = self
        self.attach(vobj)

    def attach(self, vobj):
        from pivy import coin

        self.ViewObject = vobj
        self.Object = vobj.Object

        if hasattr(self, "root") and hasattr(self, "transform"):
            self._update_scene()
            return

        if hasattr(self, "root"):
            App.Console.PrintWarning(
                "[AlignedImage VP] Altes Display ohne MatrixTransform — "
                "bitte AlignedImage neu erzeugen ({}).\n".format(
                    vobj.Object.Label if vobj.Object else "?"))

        self.root = coin.SoSeparator()
        self.transform = coin.SoMatrixTransform()
        self.geom = coin.SoSeparator()
        self.texture = coin.SoTexture2()
        self.texture.model = coin.SoTexture2.MODULATE
        self.texcoords = coin.SoTextureCoordinate2()
        self.texcoords.point.setValues([
            (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
        self.coords = coin.SoCoordinate3()
        self.coords.point.setValues([
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)])
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.FILLED
        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(1.0, 1.0, 1.0)
        faceset = coin.SoFaceSet()
        faceset.numVertices.set1Value(0, 4)

        self.geom.addChild(self.texture)
        self.geom.addChild(self.texcoords)
        self.geom.addChild(self.coords)
        self.geom.addChild(style)
        self.geom.addChild(mat)
        self.geom.addChild(faceset)
        self.root.addChild(self.transform)
        self.root.addChild(self.geom)

        vobj.addDisplayMode(self.root, "Image")
        self._update_scene()

    @staticmethod
    def _coin_matrix_from_fc(m):
        """FreeCAD row-vector matrix -> Coin (transpose)."""
        from pivy import coin

        return coin.SbMatrix(
            m.A11, m.A21, m.A31, m.A41,
            m.A12, m.A22, m.A32, m.A42,
            m.A13, m.A23, m.A33, m.A43,
            m.A14, m.A24, m.A34, m.A44,
        )

    def getDisplayModes(self, obj):
        return ["Image"]

    def getDefaultDisplayMode(self):
        return "Image"

    def setDisplayMode(self, mode):
        return mode

    def _resolve_image_path(self):
        from freecad.frametools import image_point_alignment as pa

        doc = self.Object.Document if self.Object else None
        path = pa.resolve_image_file_path(
            getattr(self.Object, "ImageFile", ""), doc)
        if path:
            return path
        source = getattr(self.Object, "SourceImage", None)
        if source is not None:
            return pa.resolve_image_file_path(
                pa._image_file_from_plane(source), doc)
        return str(getattr(self.Object, "ImageFile", "") or "")

    def _update_scene(self):
        if not hasattr(self, "coords"):
            return
        obj = self.Object
        if hasattr(obj, "WarpMatrix"):
            self.transform.matrix.setValue(
                self._coin_matrix_from_fc(obj.WarpMatrix))
        image_path = self._resolve_image_path()
        if image_path:
            self.texture.filename.setValue(image_path)

    def updateData(self, obj, prop):
        if isinstance(prop, str):
            props = [prop]
        else:
            props = list(prop)
        if not props or any(
                p in props for p in ("ImageFile", "WarpMatrix")):
            self._update_scene()

    def getIcon(self):
        return os.path.join(ICON_PATH, "image_align.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        if getattr(self, "ViewObject", None):
            self.attach(self.ViewObject)


def is_aligned_image(obj):
    if obj is None:
        return False
    proxy = getattr(obj, "Proxy", None)
    if proxy is not None:
        name = getattr(getattr(proxy, "__class__", None), "__name__", None)
        if name == "AlignedImage":
            return True
    props = getattr(obj, "PropertiesList", None)
    if props and "WarpMatrix" in props and "Corner0" in props:
        return getattr(obj, "TypeId", "") == "App::FeaturePython"
    return False
