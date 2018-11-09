# 1 create names to entities mapping
# 2 create names to (types, values) mapping

import FreeCADGui as Gui
import FreeCAD as App
import numpy as np
from scipy import sparse
from scipy.sparse import linalg


class GenericSolver():
    def __init__(self, obj, geo_obj, mesh_obj, n2e=None, n2tnv=None):
        '''A generic solver class for FEM-analysis'''
        obj.Proxy = self
        obj.addProperty("App::PropertyLink","geo_obj","Link","A shape object").geo_obj = geo_obj
        obj.addProperty("App::PropertyLink","mesh_obj","Link","A mesh object").mesh_obj = mesh_obj
        self.obj = obj
        self.n2e = n2e or {}
        self.n2tnv = n2tnv or {}

    def add_name(self, name, entities):
        if name not in self.n2e:
            self.n2e[name] = []
        self.n2e[name] += entities

    def add_typenvalues(self, name, typenvalues):
        if name not in self.n2tnv:
            self.n2tnv[name] = {}
        self.n2e[name].update(typenvalues)

    def i_add_name(self, name):
        sel = Gui.Selection.getSelectionEx()[0]
        assert sel.Object == self.geo_obj
        entities = list(sel.SubElementNames)
        assert entities
        self.add_name(name, entities)

    def solve(self):
        lambda_domains = [name for name, prop in self.n2tnv.items() if "lambda" in prop]
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
        for name in lambda_domains:
            lam = self.n2tnv[name]["lambda"]
            for element in self.n2elements[name]:
                mat_entries += self.diffuseTerm(element, lam)
        
        neumann_entries = []
        for name in neumann_domains:
            q = self.n2tnv[name]["q"]
            for element in self.n2elements[name]:
                neumann_entries += self.neumann_Term(element, q)

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

        # now we have to delete all matrix entries which are on the dirichlet boundary

        # a diagonal matrix to delete all the entries on bc_d data
        d_mod_indices = list(set(dirichlet_indices))
        bc_d_mat_1 = sparse.csr_matrix(([1] * len(d_mod_indices), (d_mod_indices, d_mod_indices)), 
                                        shape=(len(self.nodes), len(self.nodes)))
        bc_d_mat_2 = sparse.csr_matrix(([1] * len(dirichlet_indices), (dirichlet_indices, dirichlet_indices)), 
                                        shape=(len(self.nodes), len(self.nodes)))
        dia_mat = sparse.dia_matrix(([1] * len(self.nodes), 0), 
                                        shape=(len(self.nodes), len(self.nodes)))
        
        mat = (dia_mat - bc_d_mat_1) @ mat + bc_d_mat_2
        rhs = sparse.csr_matrix((len(self.nodes), 1))
        if dirichlet_data and dirichlet_indices:
            rhs += sparse.csr_matrix((dirichlet_data, (dirichlet_indices, [0] * len(dirichlet_indices))),
                                    shape=(len(self.nodes), 1))
        if neumann_entries:
            neumann_indices, neumann_data = np.array(neumann_entries).T
            neumann_indices = np.int64(neumann_indices)
            rhs += sparse.csr_matrix((neumann_data, (neumann_indices, [0] * len(neumann_indices))),
                                      shape=(len(self.nodes), 1))
        # TODO add rhs_q and rhs_r

        return linalg.spsolve(mat, rhs)

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
        y12 = y2 - y1
        y23 = y3 - y2
        y31 = y1 - y3
        x12 = x2 - x1
        x23 = x3 - x2
        x31 = x1 - x3
        area = 0.5 * abs(x12 * y31 - x31 * y12)
        B = np.array([
            [-y23, -y31, -y12],
            [ x23,  x31,  x12]]) / (2 * area)
        return area * B.T @ B

    def get_row_indices(self, element):
        return  np.array([element] * len(element))


def make_GenericSolver(geo_obj, mesh_obj, n2e, n2tnv):
    obj = App.ActiveDocument.addObject("Part::FeaturePython", "generic_solver")
    GenericSolver(obj, geo_obj, mesh_obj, n2e, n2tnv)
    # obj.ViewObject = 0
    App.ActiveDocument.recompute()
    return obj

def entity_prefix(prefix, numbers):
    return [prefix + str(i) for i in numbers]

def test():
    geo_obj = App.ActiveDocument.Fusion
    mesh_obj = App.ActiveDocument.FEMMeshNetgen
    n2e = {
        "hot": entity_prefix("Edge", [9, 3, 2, 8]),
        "cold": entity_prefix("Edge", [5, 6]),
        "material1": entity_prefix("Face", [1]),
        "material2": entity_prefix("Face", [2])
    }
    n2tnv = {
        "hot": {"T": 22., "R": 10.},
        "cold": {"T": 0.},
        "material1": {"lambda": 0.1},
        "material2": {"lambda": 2.3}
    }
    gs = make_GenericSolver(geo_obj, mesh_obj, n2e, n2tnv)
    t = gs.Proxy.solve()
    print(max(t))
    import matplotlib.pyplot as plt

    tris = np.array(gs.Proxy.n2elements["material1"]).tolist()
    tris += np.array(gs.Proxy.n2elements["material2"]).tolist()

    plt.tricontourf(gs.Proxy.nodes[:, 0],
                    gs.Proxy.nodes[:, 1], 
                    tris, t, levels=20)
    plt.colorbar()
    plt.show()

def test1():
    geo_obj = App.ActiveDocument.Fusion
    mesh_obj = App.ActiveDocument.FEMMeshNetgen
    n2e = {
        "hot": entity_prefix("Edge", [4]),
        "cold": entity_prefix("Edge", [6]),
        "material1": entity_prefix("Face", [1]),
        "material2": entity_prefix("Face", [2])
    }
    n2tnv = {
        "hot": {"T": 22.},
        "cold": {"T": 0., "R": 1.},
        "material1": {"lambda": 3.},
        "material2": {"lambda": 1.}
    }
    gs = make_GenericSolver(geo_obj, mesh_obj, n2e, n2tnv)
    t = gs.Proxy.solve()
    print(max(t))
    import matplotlib.pyplot as plt

    axes1 = plt.subplot(211)

    tris = np.array(gs.Proxy.n2elements["material1"]).tolist()
    tris += np.array(gs.Proxy.n2elements["material2"]).tolist()
    plt.tricontourf(gs.Proxy.nodes[:, 0],
                    gs.Proxy.nodes[:, 1], 
                    tris, t, levels=20)
    x = gs.Proxy.nodes[:, 0]
    order = np.argsort(x)
    ordered_x = x[order].tolist()
    ordered_t = t[order].tolist()
    ordered_x = np.array([-1., 0.] + ordered_x + [6., 7.])
    ordered_t = np.array([22., 22.] + ordered_t + [0., 0.])

    axes2 = plt.subplot(212, sharex=axes1)
    plt.plot(ordered_x, ordered_t)
    axes2.set_xlabel('x[m]')
    axes2.set_ylabel('T[°C]')
    plt.grid()
    plt.show()