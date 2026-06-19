import os

import FreeCAD as App
import Part

from freecad.frametools import ICON_PATH

REF_COLOR = (0.17, 0.47, 0.93)
MOV_COLOR = (0.93, 0.27, 0.17)
LINE_COLOR = (0.4, 0.4, 0.4)
MARKER_SIZE = 9


class FeaturePair(object):

    def __init__(self, obj):
        obj.addProperty(
            "App::PropertyVector", "RefPoint", "Feature",
            "Punkt auf Referenzbild").RefPoint = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "MovPoint", "Feature",
            "Entsprechender Punkt auf Bild 2").MovPoint = App.Vector()
        obj.Proxy = self

    def execute(self, obj):
        if obj.RefPoint.distanceToPoint(obj.MovPoint) > 1e-9:
            obj.Shape = Part.makeLine(obj.RefPoint, obj.MovPoint)
        else:
            obj.Shape = Part.Vertex(obj.RefPoint)

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

        if hasattr(self, "root"):
            return

        self.root = coin.SoSeparator()
        self.ref_sep, self.ref_coords = self._make_marker(Gui, REF_COLOR)
        self.mov_sep, self.mov_coords = self._make_marker(Gui, MOV_COLOR)

        ref_ann = coin.SoAnnotation()
        ref_ann.addChild(self.ref_sep)
        mov_ann = coin.SoAnnotation()
        mov_ann.addChild(self.mov_sep)
        self.root.addChild(ref_ann)
        self.root.addChild(mov_ann)

        self._update_markers()
        self._add_to_annotation(vobj)
        vobj.LineColor = LINE_COLOR
        vobj.LineWidth = 2

    def _add_to_annotation(self, vobj):
        if hasattr(vobj, "getAnnotation"):
            annotation = vobj.getAnnotation()
        else:
            annotation = vobj.Annotation
        annotation.addChild(self.root)

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

    def updateData(self, obj, prop):
        if isinstance(prop, str):
            props = [prop]
        else:
            props = list(prop)
        if not props or any(p in props for p in ("RefPoint", "MovPoint")):
            self._update_markers()

    def getIcon(self):
        return os.path.join(ICON_PATH, "feature_pair.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ReferenceLine(object):

    def __init__(self, obj):
        obj.addProperty(
            "App::PropertyVector", "Start", "Line", "Linienanfang").Start = App.Vector()
        obj.addProperty(
            "App::PropertyVector", "End", "Line", "Linienende").End = App.Vector()
        obj.addProperty(
            "App::PropertyLength", "TargetLength", "Line",
            "Soll-Länge").TargetLength = 0
        obj.Proxy = self

    def execute(self, obj):
        if obj.Start.distanceToPoint(obj.End) < 1e-9:
            return
        obj.Shape = Part.makeLine(obj.Start, obj.End)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ViewProviderReferenceLine(object):

    def __init__(self, vobj):
        vobj.Proxy = self

    def attach(self, vobj):
        self.ViewObject = vobj
        self.Object = vobj.Object
        vobj.LineColor = (0.1, 0.6, 0.2)
        vobj.LineWidth = 3

    def getIcon(self):
        return os.path.join(ICON_PATH, "reference_line.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def is_feature_pair(obj):
    return getattr(getattr(obj, "Proxy", None), "__class__", None).__name__ == "FeaturePair"


def is_reference_line(obj):
    return getattr(getattr(obj, "Proxy", None), "__class__", None).__name__ == "ReferenceLine"


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
            "App::PropertyLink", "SourceImage", "Image",
            "Ursprüngliches ImagePlane-Objekt")
        obj.Proxy = self

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

        if hasattr(self, "root"):
            return

        self.root = coin.SoSeparator()
        self.texture = coin.SoTexture2()
        self.texture.model = coin.SoTexture2.MODULATE
        self.texcoords = coin.SoTextureCoordinate2()
        self.texcoords.point.setValues([
            (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])
        self.coords = coin.SoCoordinate3()
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.FILLED
        mat = coin.SoMaterial()
        mat.diffuseColor.setValue(1.0, 1.0, 1.0)
        faceset = coin.SoFaceSet()
        faceset.numVertices.set1Value(0, 4)

        self.root.addChild(self.texture)
        self.root.addChild(self.texcoords)
        self.root.addChild(self.coords)
        self.root.addChild(style)
        self.root.addChild(mat)
        self.root.addChild(faceset)

        vobj.addDisplayMode(self.root, "Image")
        self._update_scene()

    def getDisplayModes(self, obj):
        return ["Image"]

    def getDefaultDisplayMode(self):
        return "Image"

    def setDisplayMode(self, mode):
        return mode

    def _resolve_image_path(self):
        path = str(self.Object.ImageFile)
        if not path:
            return ""
        if os.path.isfile(path):
            return path
        doc = self.Object.Document
        if doc and doc.FileName:
            candidate = os.path.join(os.path.dirname(doc.FileName), path)
            if os.path.isfile(candidate):
                return candidate
        return path

    def _update_scene(self):
        if not hasattr(self, "coords"):
            return
        obj = self.Object
        c0 = obj.Corner0
        c1 = obj.CornerX
        c3 = obj.CornerY
        c2 = c1 + c3 - c0
        self.coords.point.setValues([
            (c0.x, c0.y, c0.z),
            (c1.x, c1.y, c1.z),
            (c2.x, c2.y, c2.z),
            (c3.x, c3.y, c3.z),
        ])
        image_path = self._resolve_image_path()
        if image_path:
            self.texture.filename.setValue(image_path)

    def updateData(self, obj, prop):
        if isinstance(prop, str):
            props = [prop]
        else:
            props = list(prop)
        if not props or any(
                p in props for p in ("ImageFile", "Corner0", "CornerX", "CornerY")):
            self._update_scene()

    def getIcon(self):
        return os.path.join(ICON_PATH, "image_align.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def is_aligned_image(obj):
    return getattr(getattr(obj, "Proxy", None), "__class__", None).__name__ == "AlignedImage"
