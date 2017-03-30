import os
from freecad import gui
from freecad.modules.frame_tools import ICONPATH


class frame_workbench(gui.Workbench):
    """frame workbench"""

    MenuText = "frame and beams"
    ToolTip = "beam"
    Icon = os.path.join(ICONPATH, "beam.svg")
    toolbox = ["Beam", "CutMiter", "CutPlane", "CutShape", "Reload"]

    def GetClassName(self):
        return "Gui::PythonWorkbench"

    def Initialize(self):
        from freecad.modules.frame_tools import commands

        gui.addCommand('Beam', commands.Beam())
        gui.addCommand('CutMiter', commands.CutMiter())
        gui.addCommand('CutPlane', commands.CutPlane())
        gui.addCommand('CutShape', commands.CutShape())
        self.appendToolbar("Tools", self.toolbox)
        self.appendMenu("Tools", self.toolbox)

    def Activated(self):
        pass

    def Deactivated(self):
        pass


gui.addWorkbench(frame_workbench())
