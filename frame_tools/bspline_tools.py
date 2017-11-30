import numpy as np
from scipy.optimize import least_squares
import Part
import FreeCADGui as Gui


def curve_between_lines(p0, p1, v0, v1, n=None):
    if n is None:
        n = np.cross(v0, v1)
        n /= np.linalg.norm(n)
    def project_to_plane(p):
        return p - p.dot(n) * n
    tang0 = -v0 / np.linalg.norm(v0)
    tang1 = -v1 / np.linalg.norm(v1)
    knots = np.array([0., 1.])
    mults = np.array([4, 4])
    p0_proj, p1_proj, t0_proj, t1_proj = list(map(project_to_plane, [p0, p1, tang0, tang1]))
    s0, s1 = abs(np.linalg.lstsq(np.array([t0_proj, t1_proj]).T, p1_proj - p0_proj)[0])
    print(s0, s1)
    w0 = 0.56265951
    t00 = 0.65863639
    n0 = np.cross(n, t0_proj)
    n1 = np.cross(n, t1_proj)
    sn0 , sn1 = np.linalg.lstsq(np.array([n0, n1]).T, p1_proj - p0_proj)[0]
    center = p0_proj + sn0 * n0
    def min_function(params):
        w0, t00 = params
        t0 = t00 * abs(s0)
        t1 = t00 * abs(s1)
        weights = np.array([1., w0, w0, 1.])
        poles = np.array([p0_proj, p0_proj + t0 * t0_proj,
                          p1_proj + t1 * t1_proj, p1_proj])
        r = (np.linalg.norm(center - p0_proj) + np.linalg.norm(center - p1_proj)) / 2.
        bs = Part.BSplineCurve()
        bs.buildFromPolesMultsKnots(poles, mults, knots)
        for i, wi in enumerate(weights):
            bs.setWeight(i + 1, wi)
        dist = (np.array(bs.discretize(100)) - center) / r
        return sum((np.linalg.norm(dist, axis=1) - 1)**2)
    w0, t00 = least_squares(min_function, [w0, t00]).x
    t0 = t00 * abs(s0)
    t1 = t00 * abs(s1)
    weights = np.array([1, w0, w0, 1])
    poles = np.array([p0, p0 + t0 * tang0, p1 + t1 * tang1, p1])
    bs = Part.BSplineCurve()
    bs.buildFromPolesMultsKnots(poles, mults, knots)
    for i, wi in enumerate(weights):
        bs.setWeight(i + 1, wi)
    return bs


def make_nurbs_connection():
    edge1 = Gui.Selection.getSelectionEx()[0].SubObjects[0]
    edge2 = Gui.Selection.getSelectionEx()[1].SubObjects[0]
    try:
        normal = Gui.Selection.getSelectionEx()[2].SubObjects[0]
    except Exception:
        normal = None
    p11 = np.array(edge1.valueAt(edge1.FirstParameter))
    p12 = np.array(edge1.valueAt(edge1.LastParameter))
    p21 = np.array(edge2.valueAt(edge2.FirstParameter))
    p22 = np.array(edge2.valueAt(edge2.LastParameter))
    if normal is None:
        n = None
    else:
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
    if edge1.Orientation != 'Forward':
        s1 *= -1
    if edge2.Orientation != 'Forward':
        s2 *= -1

    p1 = np.array(edge1.valueAt(t1))
    p2 = np.array(edge2.valueAt(t2))
    tang1 = np.array(edge1.tangentAt(t1))
    tang2 = np.array(edge2.tangentAt(t2))
    bs = curve_between_lines(p1, p2, s1 * tang1, s2 * tang2, n)
    Part.show(bs.toShape())