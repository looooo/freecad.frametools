import os

import FreeCADGui as Gui
import FreeCAD as App
from freecad.frametools import ICON_PATH
from . import interaction, boxtools, bspline_tools
from . import fem2d


__all__ = [
    "Beam",
    "CutMiter",
    "CutPlane",
    "CutShape"]

class BaseCommand(object):

    def __init__(self):
        pass

    def GetResources(self):
        return {'Pixmap': '.svg', 'MenuText': 'Text', 'ToolTip': 'Text'}

    def IsActive(self):
        if App.ActiveDocument is None:
            return False
        else:
            return True

    def Activated(self):
        pass

    @property
    def view(self):
        return Gui.ActiveDocument.ActiveView


class Beam(BaseCommand):

    def Activated(self):
        interaction.make_beam(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'beam.svg'), 'MenuText': 'Beam', 'ToolTip': 'Create a beam'}


class CutMiter(BaseCommand):

    def Activated(self):
        interaction.make_miter_cut(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'beam_miter_cut.svg'), 'MenuText': 'Miter Cut', 'ToolTip': 'Perform miter cut of 2 beams'}


class CutPlane(BaseCommand):

    def Activated(self):
        interaction.make_plane_cut(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'beam_plane_cut.svg'), 'MenuText': 'Plane Cut', 'ToolTip': 'Cut a beam by a face of another beam'}


class CutShape(BaseCommand):

    def Activated(self):
        interaction.make_shape_cut(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'beam_shape_cut.svg'), 'MenuText': 'Shape Cut', 'ToolTip': 'Cut a beam by outer surface of another beam'}


class LinkedFace(BaseCommand):

    def Activated(self):
        boxtools.create_linked_face()

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'linked_face.svg'), 'MenuText': 'Linked Face', 'ToolTip': 'linked_face'}


class ExtrudedFace(BaseCommand):

    def Activated(self):
        boxtools.create_extruded_face()

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'extruded_face.svg'), 'MenuText': 'Extruded Face', 'ToolTip': 'extruded_face'}


class FlatFace(BaseCommand):

    def Activated(self):
        boxtools.create_flat_face()

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'linked_face.svg'), 'MenuText': 'Flat Face', 'ToolTip': 'flat_face'}


class NurbsConnection(BaseCommand):

    def Activated(self):
        bspline_tools.make_nurbs_connection()

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'nurbs_connect.svg'), 'MenuText': 'NURBS Connect', 'ToolTip': 'nurbs_connect'}


class FemSolver(BaseCommand):

    def Activated(self):
        sel = Gui.Selection.getSelection()
        fem2d.make_GenericSolver(sel[0], sel[1])
        App.ActiveDocument.recompute()

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, "generic_solver.svg"), 'MenuText': 'FEM Solver', 'ToolTip': 'fem_solver'}


class Reload():
    NOT_RELOAD = ["freecad.frametools.init_gui"]
    RELOAD = ["freecad.frametools"]
    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'reload.svg'), 'MenuText': 'Refresh', 'ToolTip': 'Refresh'}

    def IsActive(self):
        return True

    def Activated(self):
        try:
            from importlib import reload
        except ImportError:
            pass # this is python2
        import sys
        for name, mod in sys.modules.copy().items():
            for rld in self.RELOAD:
                if rld in name:
                    if mod and name not in self.NOT_RELOAD:
                        print('reload {}'.format(name))
                        reload(mod)
        from pivy import coin
