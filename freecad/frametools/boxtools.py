import FreeCAD as App
import FreeCADGui as Gui
import Part
import numpy as np

def create_extruded_face():
    sel = Gui.Selection.getSelectionEx()[0]
    base = sel.Object
    name = sel.SubElementNames[0]
    thickness = 1
    feature = App.ActiveDocument.addObject("Part::FeaturePython", "Extruded_" + name)
    ExtrudedFace(feature, base, name, thickness)
    feature.ViewObject.Proxy = 0
    App.ActiveDocument.recompute()


def create_linked_face():
    sel = Gui.Selection.getSelectionEx()[0]
    base = sel.Object
    name = sel.SubElementNames[0]
    feature = App.ActiveDocument.addObject("Part::FeaturePython", "Linked_" + name)
    LinkedFace(feature, base, name)
    feature.ViewObject.Proxy = 0
    App.ActiveDocument.recompute()


def create_flat_face():
    base = Gui.Selection.getSelection()[0]
    feature = App.ActiveDocument.addObject("Part::FeaturePython", "Flat_" + base.Name)
    FlatFace(feature, base)
    feature.ViewObject.Proxy = 0
    App.ActiveDocument.recompute()


class ExtrudedFace(object):
    def __init__(self, obj, base, name, thickness):
        self.obj = obj
        obj.Proxy = self
        obj.addProperty("App::PropertyLink", "base", "not yet", "docs").base = base
        obj.addProperty("App::PropertyString", "name", "not yet", "docs").name = name
        obj.addProperty("App::PropertyFloat", "thickness", "not yet", "docs").thickness = thickness

    def execute(self, fp):
        # check if it is a face:
        face = None
        try:
            face = fp.base.Shape.getElement(fp.name)
        except AttributeError as e:
            Warning(e)
        print(face)
        if face and isinstance(face, Part.Face):
            fp.Shape = face.extrude(face.normalAt(0, 0) * fp.thickness)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class LinkedFace(object):
    def __init__(self, obj, base, name):
        self.obj = obj
        obj.Proxy = self
        obj.addProperty("App::PropertyLink", "base", "group", "docs").base = base
        obj.addProperty("App::PropertyString", "name", "group", "docs").name = name

    def execute(self, fp):
        # check if it is a face:
        face = None
        try:
            face = fp.base.Shape.getElement(fp.name)
        except AttributeError as e:
            Warning(e)
        print(face)
        if face and isinstance(face, Part.Face):
            fp.Shape = face

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None

class FlatFace(object):
    def __init__(self, obj, base):
        self.obj = obj
        obj.Proxy = self
        obj.addProperty("App::PropertyLink", "base", "group", "docs").base = base

    def execute(self, fp):
        # get face with biggest area
        area = 0
        for i, face in enumerate(fp.base.Shape.Faces):
            if area < face.Area:
                item_nr = i
                area = face.Area
        # assume a refined shape
        face = Part.Face(fp.base.Shape.Faces[item_nr])

        normal = face.normalAt(0, 0)
        rot = App.Rotation(normal, App.Vector(0, 0, 1))
        placement = App.Placement(App.Vector(), rot)
        face = face.transformGeometry(placement.toMatrix())
        trans = App.Vector(0, 0, 0) - face.Faces[0].valueAt(0, 0)
        placement = App.Placement(trans, App.Rotation())
        face = face.transformGeometry(placement.toMatrix())
        fp.Shape = Part.Compound(face.Wires)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None
