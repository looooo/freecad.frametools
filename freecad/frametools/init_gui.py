try:
    import FreeCADGui as Gui
    import FreeCAD
except ImportError:
    print("module not loaded with freecad")

import os
from freecad.frametools import ICON_PATH

class frame_workbench(Gui.Workbench):
    '''frame workbench'''

    MenuText = 'frame and beams'
    ToolTip = 'beam'
    Icon = os.path.join(ICON_PATH, 'beam.svg')
    toolbox = ['Beam', 'CutMiter', 'CutPlane', 'CutShape', 'Reload']
    boxbox = ['LinkedFace', 'ExtrudedFace', 'FlatFace']

    def GetClassName(self):
        return 'Gui::PythonWorkbench'

    def Initialize(self):

        from freecad.frametools import commands
        Gui.addCommand('Beam', commands.Beam())
        Gui.addCommand('CutMiter', commands.CutMiter())
        Gui.addCommand('CutPlane', commands.CutPlane())
        Gui.addCommand('CutShape', commands.CutShape())
        Gui.addCommand('Reload', commands.Reload())
        Gui.addCommand('LinkedFace', commands.LinkedFace())
        Gui.addCommand('ExtrudedFace', commands.ExtrudedFace())
        Gui.addCommand('FlatFace', commands.FlatFace())
        Gui.addCommand('FlatFace', commands.NurbsConnection())

        self.appendToolbar('Frame', self.toolbox)
        self.appendMenu('Frame', self.toolbox)
        self.appendToolbar('Box', self.boxbox)
        self.appendMenu('Box', self.boxbox)

    def Activated(self):
        pass

    def Deactivated(self):
        pass

Gui.addWorkbench(frame_workbench())
