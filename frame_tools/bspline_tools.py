import numpy as np
from scipy.optimize import least_squares
import Part
import FreeCADGui as Gui


def curve_between_lines(p0, p1, v0, v1, n):
    tang0 = -v0 / np.linalg.norm(v0)
    tang1 = -v1 / np.linalg.norm(v1)
    knots = np.array([0., 1.])
    mults = np.array([4, 4])
    nt = np.cross(tang0, tang1)
    nt /= np.linalg.norm(nt)
    s0, s1, _ = np.linalg.solve(np.array([tang0, - tang1, nt]), p1 - p0)
    t_start = (abs(s0) + abs(s1)) / 3.
    w0 = 0.56265951
    def project_to_plane(p):
        return p - n * (n.dot(p))
    def min_squared_diff_curvature(params):
        '''minimizing the squared difference to the mean curvature'''
        t0 = params
        weights = np.array([1, w0, w0, 1])
        poles = [p0, p0 + t0 * tang0, p1 + t0 * tang1, p1]
        poles = [project_to_plane(p) for p in poles]
        bs = Part.BSplineCurve()
        bs.buildFromPolesMultsKnots(poles, mults, knots)
        for i, wi in enumerate(weights):
            bs.setWeight(i + 1, wi)
        edge = bs.toShape()
        ti = np.linspace(edge.FirstParameter, edge.LastParameter, 100)
        curvature = np.array([edge.curvatureAt(i) for i in ti])
        curvature -= sum(curvature) / len(curvature) # - mean
        return sum(curvature ** 2)
    res = least_squares(min_squared_diff_curvature, [t_start])
    t0 = res.x
    weights = np.array([1, w0, w0, 1])
    poles = np.array([p0, p0 + t0 * tang0, p1 + t0 * tang1, p1])
    bs = Part.BSplineCurve()
    bs.buildFromPolesMultsKnots(poles, mults, knots)
    for i, wi in enumerate(weights):
        bs.setWeight(i + 1, wi)
    return bs


def make_nurbs_connection():
    edge1 = Gui.Selection.getSelectionEx()[0].SubObjects[0]
    edge2 = Gui.Selection.getSelectionEx()[1].SubObjects[0]
    normal = Gui.Selection.getSelectionEx()[2].SubObjects[0]
    p11 = np.array(edge1.valueAt(edge1.FirstParameter))
    p12 = np.array(edge1.valueAt(edge1.LastParameter))
    p21 = np.array(edge2.valueAt(edge2.FirstParameter))
    p22 = np.array(edge2.valueAt(edge2.LastParameter))
    n = np.array(normal.tangentAt(normal.FirstParameter))
    l = [np.linalg.norm(i) for i in [p11 - p21, p11 - p22, p12 - p21, p12 - p22]]
    l_min = min(l)
    i = l.index(l_min)
    if i == 0:
        t1 = edge1.FirstParameter
        s1 = 1
        t2 = edge2.FirstParameter
        s2 = 1

    if i == 1:
        t1 = edge1.FirstParameter
        s1 = 1
        t2 = edge2.LastParameter
        s2 = -1

    if i == 2:
        t1 = edge1.LastParameter
        s1 = -1
        t2 = edge2.FirstParameter
        s2 = 1

    if i == 3:
        t1 = edge1.LastParameter
        s1 = -1
        t2 = edge2.LastParameter
        s2 = -1

    p1 = np.array(edge1.valueAt(t1))
    p2 = np.array(edge2.valueAt(t2))
    tang1 = np.array(edge1.tangentAt(t1))
    tang2 = np.array(edge2.tangentAt(t2))
    bs = curve_between_lines(p1, p2, s1 * tang1, s2 * tang2, n)
    Part.show(bs.toShape())