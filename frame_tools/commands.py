import FreeCADGui as Gui
import FreeCAD as App
from frame_tools import ICON_PATH, interaction, boxtools


def reload_package(package):
    assert(hasattr(package, '__package__'))
    fn = package.__file__
    fn_dir = os.path.dirname(fn) + os.sep
    module_visit = {fn}
    del fn

    def reload_recursive_ex(module):
        importlib.reload(module)

        for module_child in vars(module).values():
            if isinstance(module_child, types.ModuleType):
                fn_child = getattr(module_child, '__file__', None)
                if (fn_child is not None) and fn_child.startswith(fn_dir):
                    if fn_child not in module_visit:
                        # print("reloading:", fn_child, "from", module)
                        module_visit.add(fn_child)
                        reload_recursive_ex(module_child)

    return reload_recursive_ex(package)


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
        return {'Pixmap': ICON_PATH + 'beam.svg', 'MenuText': 'beam', 'ToolTip': 'beam'}


class CutMiter(BaseCommand):

    def Activated(self):
        interaction.make_miter_cut(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam_miter_cut.svg', 'MenuText': 'miter_cut', 'ToolTip': 'miter_cut'}


class CutPlane(BaseCommand):

    def Activated(self):
        interaction.make_plane_cut(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam_plane_cut.svg', 'MenuText': 'plane_cut', 'ToolTip': 'plane_cut'}


class CutShape(BaseCommand):

    def Activated(self):
        interaction.make_shape_cut(self.view)

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'beam_shape_cut.svg', 'MenuText': 'shape_cut', 'ToolTip': 'shape_cut'}


class LinkedFace(BaseCommand):

    def Activated(self):
        boxtools.create_linked_face()

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'linked_face.svg', 'MenuText': 'linked_face', 'ToolTip': 'linked_face'}


class ExtrudedFace(BaseCommand):

    def Activated(self):
        boxtools.create_extruded_face()

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'extruded_face.svg', 'MenuText': 'extruded_face', 'ToolTip': 'extruded_face'}


class FlatFace(BaseCommand):

    def Activated(self):
        boxtools.create_flat_face()

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'linked_face.svg', 'MenuText': 'flat_face', 'ToolTip': 'flat_face'}


class Reload():

    def Activated(self):
        reload(interaction)
        interaction.refresh()

    def GetResources(self):
        return {'Pixmap': ICON_PATH + 'reload.svg', 'MenuText': 'reload', 'ToolTip': 'reload'}
