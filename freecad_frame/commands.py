import FreeCADGui as Gui
import FreeCAD as App
from freecad_frame import ICON_PATH
from freecad_frame._commands import *

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
        make_beam(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam.svg', 'MenuText': 'Text', 'ToolTip': 'Text'}


class CutMiter(BaseCommand):

    def Activated(self):
        make_miter_cut(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam_miter_cut.svg', 'MenuText': 'miter_cut', 'ToolTip': 'miter_cut'}


class CutPlane(BaseCommand):

    def Activated(self):
        make_plane_cut(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam_plane_cut.svg', 'MenuText': 'plane_cut', 'ToolTip': 'plane_cut'}


class CutShape(BaseCommand):

    def Activated(self):
        make_shape_cut(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam_shape_cut.svg', 'MenuText': 'shape_cut', 'ToolTip': 'shape_cut'}