import FreeCAD as App
import FreeCADGui as Gui
import Part


def create_extruded_face():
    sel = Gui.Selection.getSelectionEx()[0]
    base = sel.Object
    name = sel.SubElementNames[0]
    thickness = 1
    feature = App.ActiveDocument.addObject("Part::FeaturePython", "Extruded_" + name)
    ExtrudedFace(feature, base, name, thickness)
    feature.ViewObject.Proxy = 0
    App.ActiveDocument.recompute()
    Gui.SendMsgToActiveView("ViewFit")


def create_linked_face():
    sel = Gui.Selection.getSelectionEx()[0]
    base = sel.Object
    name = sel.SubElementNames[0]
    thickness = 1
    feature = App.ActiveDocument.addObject("Part::FeaturePython", "Linked_" + name)
    LinkedFace(feature, base, name, thickness)
    feature.ViewObject.Proxy = 0
    App.ActiveDocument.recompute()
    Gui.SendMsgToActiveView("ViewFit")


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
    def __init__(self, obj, base, name, thickness):
        self.obj = obj
        obj.Proxy = self
        obj.addProperty("App::PropertyLink", "base", "not yet", "docs").base = base
        obj.addProperty("App::PropertyString", "name", "not yet", "docs").name = name

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

