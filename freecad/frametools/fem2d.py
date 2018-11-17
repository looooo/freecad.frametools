from PySide import QtGui, QtCore
import FreeCADGui as Gui
import FreeCAD as App
import numpy as np
from scipy import sparse
from scipy.sparse import linalg
from freecad.frametools import ICON_PATH
import os
import yaml

class PropertyEditor(object):
    widget_name = 'properties editor'

    def __init__(self, obj):
        self.obj = obj
        self.form = []
        self.base_widget = QtGui.QWidget()
        self.form.append(self.base_widget)
        self.base_widget.setWindowTitle(self.widget_name)
        self.base_widget.setLayout(QtGui.QVBoxLayout())
        self.plain_text = QtGui.QPlainTextEdit()
        self.plain_text.setTabStopWidth(30)
        self.plain_text.document().setPlainText(yaml.dump(self.obj.parameter_dict))
        self.addRefButton = QtGui.QPushButton("add entities")
        self.addRefButton.clicked.connect(self.add_entities)
        self.base_widget.layout().addWidget(self.addRefButton)
        self.base_widget.layout().addWidget(self.plain_text)

    def accept(self):
        try:
            self.obj.parameter_dict = yaml.load(self.plain_text.document().toPlainText())
        except Exception as e:
            print(e)
            return
        else:
            Gui.Control.closeDialog()

    def reject(self):
        Gui.Control.closeDialog()

    def add_entities(self):
        sel = Gui.Selection.getSelectionEx()[0]
        for i, name in enumerate(sel.SubElementNames):
            if i != 0:
                self.plain_text.insertPlainText(", ")
            self.plain_text.insertPlainText(name)

class ViewProviderGenericSolver(object):
    def __init__(self, obj):
        obj.Proxy = self
        self.obj = obj.Object

    def setupContextMenu(self, obj, menu):
        action = menu.addAction("edit parameters")
        action.triggered.connect(self.edit_dict)
        action = menu.addAction("solve")
        action.triggered.connect(self.solve)
        action = menu.addAction("show_result")
        action.triggered.connect(self.show_result)

    def edit_dict(self):
        Gui.Control.showDialog(PropertyEditor(self.obj))

    def solve(self):
        self.obj.Proxy.solve()

    def getIcon(self):
        return os.path.join(ICON_PATH, "generic_solver.svg")

    def show_result(self):
        tris = []
        for domain in self.obj.Proxy.lambda_domains: 
            tris += np.array(self.obj.Proxy.n2elements[domain]).tolist()

        from freecad.plot import Plot
        from matplotlib import pyplot as plt
        import matplotlib.cm as cm
        fig = Plot.figure()
        fig.fig.add_axes()
        ax = fig.fig.axes[0]
        ax.tricontourf(self.obj.Proxy.nodes[:, 0],
                        self.obj.Proxy.nodes[:, 1], 
                        tris, self.obj.Proxy.t, levels=100, cmap=cm.coolwarm)
        ax.grid()
        ax.set_aspect('equal')
        # fig.fig.colorbar()
        fig.show()

    def attach(self, view_obj):
        self.view_obj = view_obj
        self.obj = view_obj.Object

    def __getstate__(self):
        pass

    def __setstate__(self, state):
        pass

class GenericSolver():
    def __init__(self, obj, geo_obj, mesh_obj):
        '''A generic solver class for FEM-analysis'''
        obj.Proxy = self
        obj.addProperty("App::PropertyLink","geo_obj","Link","A shape object").geo_obj = geo_obj
        obj.addProperty("App::PropertyLink","mesh_obj","Link","A mesh object").mesh_obj = mesh_obj
        obj.addProperty('App::PropertyPythonObject', 'parameter_dict', 'parameters', 'parameters as dict')
        self.obj = obj
        self.init_parameters()

    @property
    def n2e(self):
        return self.obj.parameter_dict["entities"]

    @property
    def n2tnv(self):
        return self.obj.parameter_dict["properties"]

    def init_parameters(self):
        self.obj.parameter_dict = {"entities": {"name": ["entity1", "entity2"]}, 
                                   "properties": {"name": {"type1": "value1", "type2": "value2"}}}
                                   
    def solve(self):
        self.lambda_domains = [name for name, prop in self.n2tnv.items() if "lambda" in prop]
        dirichlet_domains = [name for name, prop in self.n2tnv.items() if ("T" in prop and not "R" in prop)]
        neumann_domains = [name for name, prop in self.n2tnv.items() if "q" in prop]
        mixed_domains = [name for name, prop in self.n2tnv.items() if ("T" in prop and "R" in prop)]
        self.n2elements = {}
        for name, entities in self.n2e.items():
            elements = []
            for entity in entities:
                if "Face" in entity:
                    elements += (self.obj.mesh_obj.FemMesh.getFacesByFace(
                                    self.obj.geo_obj.Shape.getElement(entity)))
                if "Edge" in entity:
                    elements += (self.obj.mesh_obj.FemMesh.getEdgesByEdge(
                                    self.obj.geo_obj.Shape.getElement(entity)))
            self.n2elements[name] =  [np.array(self.obj.mesh_obj.FemMesh.getElementNodes(el_index), dtype=int) - 1 for el_index in elements]
        self.nodes = np.array(list(map(list, self.obj.mesh_obj.FemMesh.Nodes.values())))
        self.nodes = self.nodes[:,:-1]  # delete z-values

        ## creating the matrix of the diffusion-matrix
        mat_entries = []
        for name in self.lambda_domains:
            lam = self.n2tnv[name]["lambda"]
            for element in self.n2elements[name]:
                mat_entries += self.diffuseTerm(element, lam)
        
        neumann_entries = []
        for name in neumann_domains:
            q = self.n2tnv[name]["q"]
            for element in self.n2elements[name]:
                neumann_entries += self.neumannTerm(element, q)

        for name in mixed_domains:
            T = self.n2tnv[name]["T"]
            R = self.n2tnv[name]["R"]
            for element in self.n2elements[name]:
                neumann_entries += self.neumannTerm(element, T / R)
                mat_entries += self.unknown_neumannTerm(element, 1. / R)

        dirichlet_indices = []
        dirichlet_data = []
        for name in dirichlet_domains:
            T = self.n2tnv[name]["T"]
            nodes = []
            for elements in self.n2elements[name]:
                nodes += elements.tolist()
            nodes = list(set(nodes)) # removing duplicated nodes
            dirichlet_indices += nodes
            dirichlet_data += [T] * len(nodes)

        # assemblying:
        row_indices, col_indices, values = np.array(mat_entries).T
        row_indices = np.int64(row_indices)
        col_indices = np.int64(col_indices)
        mat = sparse.csr_matrix((values, (row_indices, col_indices)))
        rhs = sparse.csr_matrix((len(self.nodes), 1))

        # now we have to delete all matrix entries which are on the dirichlet boundary
        # a diagonal matrix to delete all the entries on bc_d data

        if dirichlet_data and dirichlet_indices:
            d_mod_indices = list(set(dirichlet_indices))
            bc_d_mat_1 = sparse.csr_matrix(([1] * len(d_mod_indices), (d_mod_indices, d_mod_indices)), 
                                            shape=(len(self.nodes), len(self.nodes)))
            bc_d_mat_2 = sparse.csr_matrix(([1] * len(dirichlet_indices), (dirichlet_indices, dirichlet_indices)), 
                                            shape=(len(self.nodes), len(self.nodes)))
            dia_mat = sparse.dia_matrix(([1] * len(self.nodes), 0), 
                                            shape=(len(self.nodes), len(self.nodes)))
            mat = (dia_mat - bc_d_mat_1) @ mat + bc_d_mat_2

            rhs += sparse.csr_matrix((dirichlet_data, (dirichlet_indices, [0] * len(dirichlet_indices))),
                                    shape=(len(self.nodes), 1))
        if neumann_entries:
            neumann_indices, neumann_data = np.array(neumann_entries).T
            neumann_indices = np.int64(neumann_indices)
            rhs += sparse.csr_matrix((neumann_data, (neumann_indices, [0] * len(neumann_indices))),
                                      shape=(len(self.nodes), 1))
        self.t = linalg.spsolve(mat, rhs)

    def diffuseTerm(self, element, lam):
        BTB = self.get_BTB(element)
        values = lam * BTB
        row_indices = self.get_row_indices(element)
        col_indices = row_indices.T
        return np.array([row_indices.flatten(), col_indices.flatten(), values.flatten()]).T.tolist()

    def neumannTerm(self, element, q):
        HTH = self.get_HTH(element)
        values = HTH @ np.array([[q], [q]]).flatten()
        return np.array([element, values]).T.tolist()

    def unknown_neumannTerm(self, element, q):
        HTH = self.get_HTH(element) * q
        row_indices = self.get_row_indices(element)
        col_indices = row_indices.T
        return np.array([row_indices.flatten(), col_indices.flatten(), HTH.flatten()]).T.tolist()

    def get_HTH(self, element):
        x1, x2 = self.nodes[element]
        l = np.linalg.norm(x2 - x1)
        return   l * np.array([
                    [1. / 3., 1. / 6.],
                    [1. / 6., 1. / 3.]], dtype=float)

    def get_BTB(self, element):
        x1, y1, x2, y2, x3, y3 = self.nodes[element].flatten()
        y12 = y2 - y1; y23 = y3 - y2; y31 = y1 - y3
        x12 = x2 - x1; x23 = x3 - x2; x31 = x1 - x3
        area = 0.5 * abs(x12 * y31 - x31 * y12)
        B = np.array([[-y23, -y31, -y12],
                      [ x23,  x31,  x12]]) / (2 * area)
        return area * B.T @ B

    def get_row_indices(self, element):
        return  np.array([element] * len(element))

    def onDocumentRestored(self, obj):
        self.obj = obj

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None

def make_GenericSolver(geo_obj, mesh_obj):
    obj = App.ActiveDocument.addObject("App::FeaturePython", "generic_solver")
    GenericSolver(obj, geo_obj, mesh_obj)
    ViewProviderGenericSolver(obj.ViewObject)
    App.ActiveDocument.recompute()
    return obj