import os
import numpy as np
import FreeCAD as App
from Part import BSplineCurve, Wire, Face
import Part
import copy

class Screw(object):
    def __init__(self, obj):
        '''Screw'''
        print("was soll das")
        obj.addProperty("App::PropertyInteger", "threads", "screw properties", "number of threads").threads = 1
        obj.addProperty("App::PropertyLink","profile","screw properties","sketch profile of section")
        obj.addProperty("App::PropertyLength", "height", "screw properties", "total height").height = 1
        obj.addProperty("App::PropertyBool", "left_hand", "screw properties", "rotation").left_hand = False
        obj.Proxy = self
        self.Object = obj
    
    def execute(self, obj):
        if not obj.profile:
            return
        # 1: get all edges
        edges = obj.profile.Shape.Edges
        # 2: discretize edges:
        edges_discretized = []
        for e in edges:
            edges_discretized.append(np.array(e.discretize(100)))
        # assume that the profile is in xz-plane (rz)
        # so the helical projection is in the xy-plane (z=0)
        # the z-axis is the direction of the helix
        # 3: height (pitch) of profile
        height = 0
        for ed in edges_discretized:
            height = max([height, max(ed.T[2])])
        pitch = obj.threads * height
        all_edges_discretized = copy.deepcopy(edges_discretized)
        for i in range(1, obj.threads):
            for ed in edges_discretized:
                ed_new = copy.deepcopy(ed)
                ed_new.T[2] += height * i
                all_edges_discretized.append(ed_new)
        # 4: do the helical projection
        bspw = make_bspline_wire(all_edges_discretized)
        profile_coords = []
        for ed in all_edges_discretized:
            profile_coords.append(helical_projection(ed.T[0], ed.T[2], pitch, obj.left_hand)) # leftsided?
        # 5: create the bspline interpolation
        wire = make_bspline_wire(profile_coords)
        face = Face(wire)
        sign = -(int(obj.left_hand) * 2 - 1)
        obj.Shape = helicalextrusion(face, obj.height, sign * obj.height / pitch * 2 * np.pi)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None

    def attach(self):
        pass

class ViewproviderScrew(object):
    def __init__(self, obj):
        ''' Set this object to the proxy object of the actual view provider '''
        obj.Proxy = self
        self.vobj = obj

    def attach(self, vobj):
        self.vobj = vobj

    def claimChildren(self):
        return [self.vobj.Object.profile]

    def getIcon(self):
        _dir = os.path.dirname(os.path.realpath(__file__))
        return(_dir + "/" + "icons/screw.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None



def helical_projection(r, z, pitch, left_hand):
    """
    pitch: height of one helical revolution
    r: radius of points
    z: heihgt of points
    """
    sign = int(left_hand) * 2 - 1  # 1 * 2 -1 = 1, 0 * 2 - 1 = -1
    phi = 2 * sign * np.pi * z / pitch
    x = r * np.cos(phi)
    y = r * np.sin(phi)
    z = 0 * y
    return np.array([x, y, 0 * y]). T

def make_bspline_wire(pts):
    wi = []
    for i in pts:
        out = BSplineCurve()
        out.interpolate(list(map(fcvec, i)))
        wi.append(out.toShape())
    return Wire(wi)

def fcvec(x):
    if len(x) == 2:
        return(App.Vector(x[0], x[1], 0))
    else:
        return(App.Vector(x[0], x[1], x[2]))


def helicalextrusion(face, height, angle):
    """
    A helical extrusion using the BRepOffsetAPI
    face -- the face to extrude (may contain holes, i.e. more then one wires)
    height -- the height of the extrusion, normal to the face
    angle -- the twist angle of the extrusion in radians

    returns a solid
    """
    pitch = height * 2 * np.pi / abs(angle)
    radius = 10.0 # as we are only interested in the "twist", we take an arbitrary constant here
    cone_angle = 0
    direction = bool(angle < 0)
    spine = Part.makeHelix(pitch, height, radius, cone_angle, direction)
    def make_pipe(path, profile):
        """
        returns (shell, last_wire)
        """
        mkPS = Part.BRepOffsetAPI.MakePipeShell(path)
        mkPS.setFrenetMode(True) # otherwise, the profile's normal would follow the path
        mkPS.add(profile, False, False)
        mkPS.build()
        return (mkPS.shape(), mkPS.lastShape())
    shell_faces = []
    top_wires = []
    for wire in face.Wires:
        pipe_shell, top_wire = make_pipe(spine, wire)
        shell_faces.extend(pipe_shell.Faces)
        top_wires.append(top_wire)
    top_face = Part.Face(top_wires)
    shell_faces.append(top_face)
    shell_faces.append(face) # the bottom is what we extruded
    shell = Part.makeShell(shell_faces)
    #shell.sewShape() # fill gaps that may result from accumulated tolerances. Needed?
    #shell = shell.removeSplitter() # refine. Needed?
    return Part.makeSolid(shell)

def make_face(edge1, edge2):
    v1, v2 = edge1.Vertexes
    v3, v4 = edge2.Vertexes
    e1 = Wire(edge1)
    e2 = LineSegment(v1.Point, v3.Point).toShape().Edges[0]
    e3 = edge2
    e4 = LineSegment(v4.Point, v2.Point).toShape().Edges[0]
    w = Wire([e3, e4, e1, e2])
    return(Face(w))
