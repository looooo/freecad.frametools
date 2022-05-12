from __future__ import division
import FreeCAD as App
import FreeCADGui as Gui
import numpy as np
import os
import Part
from PySide import QtGui

from . import beamobj

__all__ = [
    "make_beam",
    "make_miter_cut",
    "make_plane_cut",
    "make_shape_cut"]


def refresh():
    reload(beamobj)


class make_beam(object):

    def __init__(self, view):
        self.profile = None
        self.path = None
        self.view = view
        App.Console.PrintMessage("choose the profile\n")
        Gui.Selection.clearSelection()

        self.observer = Gui.Selection.addObserver(self)

        self.form = Gui.PySideUic.loadUi(os.path.join(os.path.dirname(__file__), "make_beam.ui"))
        self.form.buttonProfile.setChecked(True)
        self.form.buttonProfile.clicked.connect(self.selectProfile)
        self.form.buttonPaths.clicked.connect(self.selectPaths)
        self.form.buttonCreate.clicked.connect(self.createBeam)
        self.form.buttonCreate.setEnabled(False)

        Gui.Control.showDialog(self)

    def selectProfile(self):
        if self.form.buttonProfile.isChecked():
            self.form.buttonPaths.setChecked(False)
            Gui.Selection.clearSelection()

    def selectPaths(self):
        if self.form.buttonPaths.isChecked():
            self.form.buttonProfile.setChecked(False)
            Gui.Selection.clearSelection()

    def addSelection(self, document, object, subobject, pos):
        if self.form.buttonProfile.isChecked():
            self.profile = App.getDocument(document).getObject(object)
            self.form.lineProfile.setText(self.profile.Label)
            self.form.buttonProfile.setChecked(False)
            self.form.buttonPaths.setChecked(True)
            Gui.Selection.clearSelection()
            App.Console.PrintMessage("choose_path\n")
        else:
            if subobject != "":
                Gui.Selection.clearSelection()
                self.path = (document, object, subobject)

            (document, object, subobject) = self.path
            obj = App.getDocument(document).getObject(object)

            if obj != None:
                self.form.path.setText("{}.{}".format(obj.Label, subobject))

        self.form.buttonCreate.setEnabled((self.profile != None) and (self.path != None))

    def createBeam(self):
        (document, object, subobject) = self.path
        path_sketch = App.getDocument(document).getObject(object)

        if object != None:
            a = App.ActiveDocument.addObject("Part::FeaturePython", "beam")
            beamobj.Beam(a, self.profile, path_sketch, subobject)
            beamobj.ViewProviderBeam(a.ViewObject)
            a.exdent_1 = self.form.extent1.property("value")
            a.exdent_2 = self.form.extent2.property("value")
            a.Rotation = self.form.rotation.property("value")

        App.ActiveDocument.recompute()

    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Close)

    def reject(self):
        Gui.Selection.removeObserver(self)
        Gui.Control.closeDialog()


class make_miter_cut(object):

    def __init__(self, view):
        self.view = view
        self.beam_1 = None
        self.beam_2 = None

        Gui.Selection.addObserver(self)

        self.form = Gui.PySideUic.loadUi(os.path.join(os.path.dirname(__file__), "miter-cut.ui"))
        self.form.setProperty("windowTitle", "Miter Cut")
        self.form.buttonBeam1.setChecked(True)
        self.form.buttonBeam1.clicked.connect(self.selectBeam1)
        self.form.buttonBeam2.clicked.connect(self.selectBeam2)
        self.form.buttonCut.clicked.connect(self.createCut)
        self.form.buttonCut.setEnabled(False)

        items = Gui.Selection.getSelectionEx()
        if (len(items) > 0) and (items[0].Object.Proxy != None) \
           and ((items[0].Object.Proxy.__class__.__name__ == 'Beam') or (items[0].Object.Proxy.__class__.__name__ == 'CBeam')):
            self.addSelection(items[0].DocumentName, items[0].ObjectName, None, None)
            if (len(items) > 1) and (items[1].Object.Proxy != None) \
               and ((items[1].Object.Proxy.__class__.__name__ == 'Beam') or (items[1].Object.Proxy.__class__.__name__ == 'CBeam')):
                self.addSelection(items[1].DocumentName, items[1].ObjectName, None, None)

        Gui.Control.showDialog(self)
        Gui.Selection.clearSelection()

    def set_miter_cut_obj(self, obj, beam):
        if not obj.cut_type:
            obj.cut_obj = beam
            obj.cut_type = "miter"
            return
        return False

    def selectBeam1(self):
        if self.form.buttonBeam1.isChecked():
            self.form.buttonBeam2.setChecked(False)
            Gui.Selection.clearSelection()

    def selectBeam2(self):
        if self.form.buttonBeam2.isChecked():
            self.form.buttonBeam1.setChecked(False)
            Gui.Selection.clearSelection()

    def addSelection(self, document, object, subobject, pos):
        obj = App.getDocument(document).getObject(object)
        name = "{}.{}".format(document, obj.Label)

        if self.form.buttonBeam1.isChecked():
            self.beam_1 = obj
            self.form.beam1.setText(name)
            self.form.buttonBeam2.setChecked(True)
            self.selectBeam2()
        elif self.form.buttonBeam2.isChecked():
            self.beam_2 = obj
            self.form.beam2.setText(name)

        messages = ""
        if (self.beam_1 != None) and (self.beam_1.Proxy.__class__.__name__ != "Beam") and (self.beam_1.Proxy.__class__.__name__ != "CBeam"):
            messages = messages + "The object {}.{} is not of beam type\n".format(self.beam_1.Document.Name, self.beam_1.Label)
        if (self.beam_2 != None) and (self.beam_2.Proxy.__class__.__name__ != "Beam") and (self.beam_2.Proxy.__class__.__name__ != "CBeam"):
            messages = messages + "The object {}.{} is not of beam type\n".format(self.beam_2.Document.Name, self.beam_2.Label)

        self.form.validationMessage.setText(messages)
        self.form.buttonCut.setEnabled((self.beam_1 != None) and (self.beam_2 != None) and (self.beam_1 != self.beam_2) and (len(messages) == 0))

    def createCut(self):
        a = App.ActiveDocument.addObject(
            "Part::FeaturePython", "miter_beam")
        beamobj.CBeam(a, self.beam_1)
        self.set_miter_cut_obj(a, self.beam_2)
        beamobj.ViewProviderCBeam(a.ViewObject)

        a = App.ActiveDocument.addObject(
            "Part::FeaturePython", "miter_beam")
        beamobj.CBeam(a, self.beam_2)
        self.set_miter_cut_obj(a, self.beam_1)
        beamobj.ViewProviderCBeam(a.ViewObject)

        self.form.buttonBeam1.setChecked(True)
        self.selectBeam1()
        App.ActiveDocument.recompute()

    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Close)

    def reject(self):
        Gui.Selection.removeObserver(self)
        Gui.Control.closeDialog()


def to_np(app_vec):
    return np.array([app_vec.x, app_vec.y, app_vec.z])


class make_plane_cut(object):

    def __init__(self, view):
        self.view = view
        self.beam = None
        self.cut_beam = None
        self.cut_face = None
        Gui.Selection.addObserver(self)

        self.form = Gui.PySideUic.loadUi(os.path.join(os.path.dirname(__file__), "plane-cut.ui"))
        self.form.setProperty("windowTitle", "Plane Cut")
        self.form.buttonBeam.setChecked(True)
        self.form.buttonBeam.clicked.connect(self.selectBeam)
        self.form.buttonFace.clicked.connect(self.selectFace)
        self.form.buttonCut.clicked.connect(self.createCut)
        self.form.buttonCut.setEnabled(False)

        items = Gui.Selection.getSelectionEx()
        if (len(items) > 0) and (items[0].Object.Proxy != None) and ((items[0].Object.Proxy.__class__.__name__ == 'Beam') or (items[0].Object.Proxy.__class__.__name__ == 'CBeam')):
            self.addSelection(items[0].DocumentName, items[0].ObjectName, None, None)
            if (len(items) > 1) and (items[1].Object.Proxy != None) \
               and ((items[1].Object.Proxy.__class__.__name__ == 'Beam') or (items[1].Object.Proxy.__class__.__name__ == 'CBeam')) \
               and (len(items[1].SubElementNames) > 0):
                self.addSelection(items[1].DocumentName, items[1].ObjectName, items[1].SubElementNames[0], None)

        Gui.Control.showDialog(self)
        Gui.Selection.clearSelection()

    def set_cut_obj(self, obj, beam, face):
        if not obj.cut_type:
            obj.cut_obj = beam
            obj.cut_type = "cut"
            obj.cut_obj_name = face
            return
        return False

    def selectBeam(self):
        if self.form.buttonBeam.isChecked():
            self.form.buttonFace.setChecked(False)
            Gui.Selection.clearSelection()

    def selectFace(self):
        if self.form.buttonFace.isChecked():
            self.form.buttonBeam.setChecked(False)
            Gui.Selection.clearSelection()

    def addSelection(self, document, object, subobject, pos):
        obj = App.getDocument(document).getObject(object)

        if self.form.buttonBeam.isChecked():
            self.beam = obj
            self.form.beam.setText("{}.{}".format(document, obj.Label))
            self.form.buttonFace.setChecked(True)
            self.selectFace()
        elif self.form.buttonFace.isChecked() and (subobject != None):
            self.cut_beam = obj
            self.cut_face = subobject
            self.form.face.setText("{}.{}.{}".format(document, obj.Label, subobject))

        messages = ""
        if (self.beam != None) and (self.beam.Proxy.__class__.__name__ != "Beam") and (self.beam.Proxy.__class__.__name__ != "CBeam"):
            messages = messages + "The object {}.{} is not of beam type\n".format(self.beam.Document.Name, self.beam.Label)
        if (self.cut_beam != None) and (self.cut_beam.Proxy.__class__.__name__ != "Beam") and (self.cut_beam.Proxy.__class__.__name__ != "CBeam"):
            messages = messages + "The object {}.{} is not of beam type\n".format(self.cut_beam.Document.Name, self.cut_beam.Name)

        self.form.validationMessage.setText(messages)
        self.form.buttonCut.setEnabled((self.beam != None) and (self.cut_beam != None) and (self.beam != self.cut_beam) and (len(messages) == 0))

    def createCut(self):
        a = App.ActiveDocument.addObject(
            "Part::FeaturePython", "cut_beam")
        beamobj.CBeam(a, self.beam)
        self.set_cut_obj(a, self.cut_beam, self.cut_face)
        beamobj.ViewProviderCBeam(a.ViewObject)

        self.form.buttonBeam.setChecked(True)
        self.selectBeam()
        App.ActiveDocument.recompute()

    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Close)

    def reject(self):
        Gui.Selection.removeObserver(self)
        Gui.Control.closeDialog()


class make_shape_cut(object):

    def __init__(self, view):
        self.view = view
        self.beam_1 = None
        self.beam_2 = None

        Gui.Selection.addObserver(self)

        self.form = Gui.PySideUic.loadUi(os.path.join(os.path.dirname(__file__), "shape-cut.ui"))
        self.form.setProperty("windowTitle", "Shape Cut")
        self.form.buttonBeam1.setChecked(True)
        self.form.buttonBeam1.clicked.connect(self.selectBeam1)
        self.form.buttonBeam2.clicked.connect(self.selectBeam2)
        self.form.buttonCut.clicked.connect(self.createCut)
        self.form.buttonCut.setEnabled(False)

        items = Gui.Selection.getSelectionEx()
        if (len(items) > 0) and (items[0].Object.Proxy != None) \
           and ((items[0].Object.Proxy.__class__.__name__ == 'Beam') or (items[0].Object.Proxy.__class__.__name__ == 'CBeam')):
            self.addSelection(items[0].DocumentName, items[0].ObjectName, None, None)
            if (len(items) > 1) and (items[1].Object.Proxy != None) \
               and ((items[1].Object.Proxy.__class__.__name__ == 'Beam') or (items[1].Object.Proxy.__class__.__name__ == 'CBeam')):
                self.addSelection(items[1].DocumentName, items[1].ObjectName, None, None)

        Gui.Control.showDialog(self)
        Gui.Selection.clearSelection()

    def set_cut_obj(self, obj, beam):
        if not obj.cut_type:
            obj.cut_obj = beam
            obj.cut_type = "shape_cut"
            return
        return False

    def selectBeam1(self):
        if self.form.buttonBeam1.isChecked():
            self.form.buttonBeam2.setChecked(False)
            Gui.Selection.clearSelection()

    def selectBeam2(self):
        if self.form.buttonBeam2.isChecked():
            self.form.buttonBeam1.setChecked(False)
            Gui.Selection.clearSelection()

    def addSelection(self, document, object, subobject, pos):
        obj = App.getDocument(document).getObject(object)
        name = "{}.{}".format(document, obj.Label)

        if self.form.buttonBeam1.isChecked():
            self.beam_1 = obj
            self.form.beam1.setText(name)
            self.form.buttonBeam2.setChecked(True)
            self.selectBeam2()
        elif self.form.buttonBeam2.isChecked():
            self.beam_2 = obj
            self.form.beam2.setText(name)

        messages = ""
        if (self.beam_1 != None) and (self.beam_1.Proxy.__class__.__name__ != "Beam") and (self.beam_1.Proxy.__class__.__name__ != "CBeam"):
            messages = messages + "The object {}.{} is not of beam type\n".format(self.beam_1.Document.Name, self.beam_1.Label)
        if (self.beam_2 != None) and (self.beam_2.Proxy.__class__.__name__ != "Beam") and (self.beam_2.Proxy.__class__.__name__ != "CBeam"):
            messages = messages + "The object {}.{} is not of beam type\n".format(self.beam_2.Document.Name, self.beam_2.Label)

        self.form.validationMessage.setText(messages)
        self.form.buttonCut.setEnabled((self.beam_1 != None) and (self.beam_2 != None) and (self.beam_1 != self.beam_2) and (len(messages) == 0))

    def createCut(self):
        a = App.ActiveDocument.addObject(
            "Part::FeaturePython", "shape_cut_beam")
        beamobj.CBeam(a, self.beam_1)
        self.set_cut_obj(a, self.beam_2)
        beamobj.ViewProviderCBeam(a.ViewObject)

        Gui.Selection.clearSelection()
        self.form.buttonBeam1.setChecked(True)
        self.selectBeam1()
        App.ActiveDocument.recompute()

    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Close)

    def reject(self):
        Gui.Selection.removeObserver(self)
        Gui.Control.closeDialog()
