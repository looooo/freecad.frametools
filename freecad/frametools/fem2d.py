# 1 create names to entities mapping
# 2 create names to (types, values) mapping

import FreeCADGui as Gui


class GenericSolver():
    def __init__(self, obj, geo_obj, mesh_obj):
        '''A generic solver class for FEM-analysis'''
        obj.Proxy = self
        obj.addProperty("App::PropertyLink","geo_obj","Link","A shape object").geo_obj = geo_obj
        obj.addProperty("App::PropertyLink","mesh_obj","Link","A mesh object").mesh_obj = mesh_obj
        self.names2entities = {}
        self.names2typenvalues = {}

    def add_name(self, name, entities):
        if name not in self.names2entities:
            self.names2entities[name] = []
        self.names2entities[name] += entities

    def add_typenvalues(self, name, typenvalues):
        if name not in self.names2typenvalues:
            self.names2typenvalues[name] = {}
        self.names2entities[name].update(typenvalues)

    def i_add_name(self, name):
        sel = Gui.Selection.getSelectionEx()[0]
        entities = list(sel.SubElementNames)
        assert entities
        self.add_name(name, entities)