import os

import FreeCADGui as Gui
import FreeCAD as App
from freecad.frametools import ICON_PATH
from . import interaction, boxtools, bspline_tools
from . import fem2d
from . import screw_maker
from . import image_tools


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

class ScrewMaker(BaseCommand):

    def Activated(self):
        a = App.ActiveDocument.addObject("Part::FeaturePython", "screw")
        screw_maker.Screw(a)
        screw_maker.ViewproviderScrew(a.ViewObject)

    def GetResources(self):
        return {'Pixmap': os.path.join(ICON_PATH, 'screw.svg'), 'MenuText': 'create Screw', 'ToolTip': 'create Screw'}



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


class AlignedImage(BaseCommand):

    def Activated(self):
        image_tools.convert_selected_to_aligned_images()

    def GetResources(self):
        return {
            'Pixmap': os.path.join(ICON_PATH, 'image_align.svg'),
            'MenuText': 'Aligned Image',
            'ToolTip': 'Draft/Image in AlignedImage mit Coin-Darstellung umwandeln',
        }


class FeaturePair(BaseCommand):

    def Activated(self):
        image_tools.create_feature_pair()

    def GetResources(self):
        return {
            'Pixmap': os.path.join(ICON_PATH, 'feature_pair.svg'),
            'MenuText': 'Feature Pair',
            'ToolTip': 'Feature-Paar aus zwei Punkten erstellen',
        }


class ReferenceLine(BaseCommand):

    def Activated(self):
        image_tools.create_reference_line()

    def GetResources(self):
        return {
            'Pixmap': os.path.join(ICON_PATH, 'reference_line.svg'),
            'MenuText': 'Reference Line',
            'ToolTip': 'Referenzlinie mit Soll-Länge erstellen',
        }


class ImageOverlay(BaseCommand):

    def Activated(self):
        image_tools.overlay_images()

    def GetResources(self):
        return {
            'Pixmap': os.path.join(ICON_PATH, 'image_overlay.svg'),
            'MenuText': 'Image Overlay',
            'ToolTip': 'Zwei Bilder anhand von Feature-Paaren überlagern',
        }


class ScaleSolver(BaseCommand):

    def Activated(self):
        image_tools.solve_reference_lines()

    def GetResources(self):
        return {
            'Pixmap': os.path.join(ICON_PATH, 'scale_solver.svg'),
            'MenuText': 'Scale Solver',
            'ToolTip': 'Orientierung/Skalierung anhand von Referenzlinien berechnen',
        }


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
