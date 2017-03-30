import os
from freecad import app
from freecad import gui
from freecad.modules.frame_tools import ICONPATH, interaction


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
        if app.ActiveDocument is None:
            return False
        else:
            return True

    def Activated(self):
        pass

    @property
    def view(self):
        return gui.ActiveDocument.ActiveView


class Beam(BaseCommand):

    def Activated(self):
        interaction.make_beam(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICONPATH, 'beam.svg'), 'MenuText': 'beam', 'ToolTip': 'beam'}


class CutMiter(BaseCommand):

    def Activated(self):
        interaction.make_miter_cut(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICONPATH, 'beam_miter_cut.svg'), 
                'MenuText': 'miter_cut', 'ToolTip': 'miter_cut'}


class CutPlane(BaseCommand):

    def Activated(self):
        interaction.make_plane_cut(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICONPATH, 'beam_plane_cut.svg'), 
                'MenuText': 'plane_cut', 'ToolTip': 'plane_cut'}


class CutShape(BaseCommand):

    def Activated(self):
        interaction.make_shape_cut(self.view)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICONPATH, 'beam_shape_cut.svg'), 
                'MenuText': 'shape_cut', 'ToolTip': 'shape_cut'}
