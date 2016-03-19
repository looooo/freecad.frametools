from __future__ import division
import FreeCAD as App
import FreeCADGui as Gui
import numpy as np
import Part

import beamobj

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
        self.midpoint = None
        self.path = None
        self.n = None
        self.view = view
        App.Console.PrintMessage("choose the profile\n")
        Gui.Selection.clearSelection()
        self.klick_event = self.view.addEventCallback(
            "SoMouseButtonEvent", self.choose_profile)

    def choose_profile(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelection()
            if len(sel) > 0:
                self.profile = sel[0]
                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)
                self.klick_event = self.view.addEventCallback(
                    "SoMouseButtonEvent", self.choose_path)
                App.Console.PrintMessage("choose_path\n")

    def choose_path(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelectionEx()
            if sel:
                path_sketch = sel[0].Object
                path_name = sel[0].SubElementNames[0]

                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)

                a = App.ActiveDocument.addObject("Part::FeaturePython", "beam")
                beamobj.Beam(a, self.profile, path_sketch, path_name)
                beamobj.ViewProviderBeam(a.ViewObject)
                App.ActiveDocument.recompute()
                App.Console.PrintMessage("end of tube tool\n")


class make_miter_cut(object):

    def __init__(self, view):
        self.view = view
        App.Console.PrintMessage("choose a beam\n")
        self.klick_event = self.view.addEventCallback(
            "SoMouseButtonEvent", self.choose_beam_1)

    def set_miter_cut_obj(self, obj, beam):
        if not obj.cut_type:
            obj.cut_obj = beam
            obj.cut_type = "miter"
            return
        return False

    def choose_beam_1(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelection()
            if len(sel) > 0:
                self.beam_1 = sel[0]
                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)
                self.klick_event = self.view.addEventCallback(
                    "SoMouseButtonEvent", self.choose_beam_2)
                App.Console.PrintMessage("choose another beam\n")

    def choose_beam_2(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelection()
            if len(sel) > 0:
                self.beam_2 = sel[0]
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

                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)
                App.ActiveDocument.recompute()


def to_np(app_vec):
    return np.array([app_vec.x, app_vec.y, app_vec.z])


class make_plane_cut(object):

    def __init__(self, view):
        self.view = view
        App.Console.PrintMessage("choose a beam\n")
        self.klick_event = self.view.addEventCallback(
            "SoMouseButtonEvent", self.choose_beam)

    def set_cut_obj(self, obj, beam, face):
        if not obj.cut_type:
            obj.cut_obj = beam
            obj.cut_type = "cut"
            obj.cut_obj_name = face
            return
        return False

    def choose_beam(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelection()
            if len(sel) > 0:
                self.beam = sel[0]
                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)
                self.klick_event = self.view.addEventCallback(
                    "SoMouseButtonEvent", self.choose_plane)
                App.Console.PrintMessage("choose acutting plane\n")

    def choose_plane(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelectionEx()
            if len(sel) > 0:
                self.cut_beam = sel[0].Object
                self.cut_face = sel[0].SubElementNames[0]
                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)

                a = App.ActiveDocument.addObject(
                    "Part::FeaturePython", "cut_beam")
                beamobj.CBeam(a, self.beam)
                self.set_cut_obj(a, self.cut_beam, self.cut_face)
                beamobj.ViewProviderCBeam(a.ViewObject)
                App.ActiveDocument.recompute()


class make_shape_cut(object):

    def __init__(self, view):
        self.view = view
        App.Console.PrintMessage("choose a beam\n")
        self.klick_event = self.view.addEventCallback(
            "SoMouseButtonEvent", self.choose_beam_1)

    def set_cut_obj(self, obj, beam):
        if not obj.cut_type:
            obj.cut_obj = beam
            obj.cut_type = "shape_cut"
            return
        return False

    def choose_beam_1(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelection()
            if len(sel) > 0:
                self.beam_1 = sel[0]
                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)
                self.klick_event = self.view.addEventCallback(
                    "SoMouseButtonEvent", self.choose_beam_2)
                App.Console.PrintMessage("choose a cutting beam\n")

    def choose_beam_2(self, cb):
        if cb["State"] == "DOWN":
            sel = Gui.Selection.getSelection()
            if len(sel) > 0:
                self.beam_2 = sel[0]
                a = App.ActiveDocument.addObject(
                    "Part::FeaturePython", "shape_cut_beam")
                beamobj.CBeam(a, self.beam_1)
                self.set_cut_obj(a, self.beam_2)
                beamobj.ViewProviderCBeam(a.ViewObject)

                Gui.Selection.clearSelection()
                self.view.removeEventCallback(
                    "SoMouseButtonEvent", self.klick_event)
                App.ActiveDocument.recompute()
