try:
    import FreeCADGui as Gui
    import FreeCAD
except ImportError:
    print("module not loaded with freecad")

from freecad_frame import *
Gui.addIconPath(ICON_PATH)

class frame_workbench(Workbench):
    """frame workbench"""

    MenuText = "frame and beams"
    ToolTip = "beam"
    Icon = "beam.svg"
    toolbox = ["Beam", "CutMiter", "CutPlane", "CutShape"]

    def GetClassName(self):
        return "Gui::PythonWorkbench"

    def Initialize(self):

        from freecad_frame import commands
        Gui.addCommand('Beam', commands.Beam())
        Gui.addCommand('CutMiter', commands.CutMiter())
        Gui.addCommand('CutPlane', commands.CutPlane())
        Gui.addCommand('CutShape', commands.CutShape())
        self.appendToolbar("Tools", self.toolbox)
        self.appendMenu("Tools", self.toolbox)

    def Activated(self):
        pass

    def Deactivated(self):
        pass

Gui.addWorkbench(frame_workbench())
