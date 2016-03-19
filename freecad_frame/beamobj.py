from __future__ import division
import copy

import os
import numpy as np

import FreeCAD as App
import Part

from freecad_frame import ICON_PATH

__all__ = [
    "Beam",
    "CBeam",
    "ViewProviderBeam",
    "ViewProviderCBeam"]


class Beam():
    def __init__(self, obj, profile, path, path_name):
        '''Beam: representing a straight extrusion of a profile'''
        print("was soll das")
        obj.addProperty("App::PropertyString", "type", "Beam", "type of the object").type = "beam"
        obj.addProperty("App::PropertyLink","profile","Beam","sketch profile of the beam").profile = profile
        obj.addProperty("App::PropertyLink","path","Beam","path of the beam").path = path
        obj.addProperty("App::PropertyString","path_name","Beam", "name of beam line").path_name = path_name
        obj.addProperty("App::PropertyDistance", "exdent_1", "Beam", "exdent side 1").exdent_1 = "0 mm"
        obj.addProperty("App::PropertyDistance", "exdent_2", "Beam", "exdent side 2").exdent_2 = "0 mm"
        obj.addProperty("App::PropertyAngle", "Rotation", "Beam", "rotation of the profile").Rotation = 0    
        obj.setEditorMode("path_name", 1)
        obj.Proxy = self
        self.Object = obj

    def initialize(self):
        print("initialize ...")

    @property
    def profile(self):
        if isinstance(self.Object.profile.Shape, Part.Face):
            # create a copy of the face
            return Part.Face(self.Object.profile.Shape)
        wires = copy.copy(self.Object.profile.Shape.Wires)
        # check boundingbox diagonals to get outer shape
        if len(wires) > 1:
            diagonals = [wire.BoundBox.DiagonalLength for wire in wires]
            pos = diagonals.index(max(diagonals))
            # reaordering
            external_wire = wires[pos]
            wires.pop(pos)
            wires.insert(0, external_wire)
            # check face oriendation
            orientation = wires[0].Orientation
            normal = Part.Face(wires[0]).normalAt(0, 0)
            for wire in wires[1:]:
                if Part.Face(wire).normalAt(0, 0).dot(normal) < 0:
                    wire.reverse()

        face = Part.Face(wires)
        return face

    @property
    def outer_shape(self):
        path, a, b, n = self.path_a_b_n
        profile = Part.Face(self.profile.Wires[0])
        new_profile = self.transform_to_n(profile, a - n * self.Object.exdent_1.Value, n)
        return new_profile.extrude(App.Vector(b - a) + n * (self.Object.exdent_1.Value + self.Object.exdent_2.Value))

    def attach(self, fp):
        print("hi")
        fp.Proxy.Object = fp

    def execute(self, fp):
        '''Do something when doing a recomputation, this method is mandatory'''
        fp.Proxy.Object = fp
        path, a, b, n = self.path_a_b_n
        new_profile = self.transform_to_n(self.profile, a - n * fp.exdent_1.Value, n)
        fp.Shape = new_profile.extrude(App.Vector(b - a) + n * (fp.exdent_1.Value + fp.exdent_2.Value))

    def transform_to_n(self, profile, p, n):
        '''transform a profile (Shape):
                rotating from  t to n
                and translating from 0 to p'''


        n.normalize()
        t = sketch_normal(n, self.Object.path)
        t.normalize()
        v = n.cross(t)
        v.normalize()

        rot_mat = App.Matrix()
        rot_mat.A11 = t.x
        rot_mat.A21 = t.y
        rot_mat.A31 = t.z
        rot_mat.A12 = v.x
        rot_mat.A22 = v.y
        rot_mat.A32 = v.z
        rot_mat.A13 = n.x
        rot_mat.A23 = n.y
        rot_mat.A33 = n.z        
        rot_mat.A14 = p.x
        rot_mat.A24 = p.y
        rot_mat.A34 = p.z
        if hasattr(self.Object.profile, "Sources"):
            translation = self.Object.profile.Sources[0].Placement.Base
            #translation += self.Object.profile.Placement.Base
        else:
            translation = self.Object.profile.Placement.Base
        print(self.Object.profile.Placement)
        print(profile.Placement)
        print(profile.Placement)
        rot_mat_2 = profile.Placement.toMatrix().inverse()
        profile.Placement.Base -= translation
        rot_mat_1 = App.Matrix()
        rot_mat_1.rotateZ(np.deg2rad(self.Object.Rotation.Value))

        placement = rot_mat.multiply(rot_mat_1.multiply(rot_mat_2))
        profile.Placement = App.Placement(placement).multiply(profile.Placement)

        return profile

    # @property
    # def midpoint(self):
    #     profile_midpoint = filter(lambda x: isinstance(x, App.Base.Vector), self.Object.profile.Geometry)
    #     if len(profile_midpoint) == 0 or profile_midpoint is None:
    #         profile_midpoint = App.Vector(0, 0, 0)
    #     else:
    #         profile_midpoint = profile_midpoint[0]
    #     return profile_midpoint

    @property
    def path_a_b_n(self):
        path = self.Object.path.Shape.getElement(self.Object.path_name)
        a, b = path.Vertexes
        n = b.Point - a.Point
        n.normalize()
        return (path, a.Point, b.Point, n)

    @property
    def profile_bound_box(self):
        fac = 0.3
        bb = self.Object.profile.Shape.BoundBox
        xmax = bb.XMax + bb.XLength * fac
        xmin = bb.XMin - bb.XLength * fac
        ymax = bb.YMax + bb.YLength * fac
        ymin = bb.YMin - bb.YLength * fac
        pol = Part.makePolygon(
            [App.Vector(xmax, ymax, 0),
             App.Vector(xmax, ymin, 0),
             App.Vector(xmin, ymin, 0),
             App.Vector(xmin, ymax, 0),
             App.Vector(xmax, ymax, 0)])
        path, a, b, n = self.path_a_b_n
        return self.transform_to_n(pol, a, n)

    def project_profile_bound_box(self, plane_n, plane_p):
        bb = self.profile_bound_box.Vertexes
        vectors = [v.Point for v in bb]
        path, a, b, n = self.path_a_b_n
        projected = [project_to_plane(plane_p, plane_n, v, n) for v in vectors]
        projected.append(projected[0])
        pol = Part.makePolygon(projected)
        return pol

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None

    def attach(self):
        pass


class ViewProviderBeam:
    def __init__(self, obj):
        ''' Set this object to the proxy object of the actual view provider '''
        obj.Proxy = self

    def attach(self, vobj):
        self.vobj = vobj

    def getIcon(self):
        _dir = os.path.dirname(os.path.realpath(__file__))
        return(_dir + "/" + "icons/beam.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class CBeam(object):
    def __init__(self, obj, beam):
        "'''Beam: representing a straight extrusion of a profile'''"
        obj.addProperty("App::PropertyString", "type", "Beam", "type of the object").type = "c_beam"
        obj.addProperty("App::PropertyLink","this_beam","Beam","sketch profile of the beam").this_beam = beam
        obj.addProperty("App::PropertyLink","cut_obj","Cut","link to cut obj")
        obj.addProperty("App::PropertyString","cut_type","Cut","type of cut obj")
        obj.addProperty("App::PropertyString","cut_obj_name","Cut","type of cut obj")
        obj.addProperty("App::PropertyBool", "cut", "Beam", "show the cut").cut = True
        self.Object = obj
        obj.Proxy = self

    def attach(self, fp):
        self.Object = fp

    def execute(self, fp):
        fp.Proxy.Object = fp
        if self.Object.cut:
            if fp.cut_type == "miter":
                fp.Shape = self.miter_cut(self.Object.cut_obj)
            elif fp.cut_type == "cut":
                face = self.Object.cut_obj.Shape.getElement(self.Object.cut_obj_name)
                fp.Shape = self.face_cut(self.Object.cut_obj, face)
            elif fp.cut_type == "shape_cut":
                fp.Shape = self.shape_cut(self.Object.cut_obj)
        else:
            fp.Shape = fp.this_beam.Shape

    def find_top_beam(self, beam):
        if "path_a_b_n" in dir(beam.Proxy):
            beam.Proxy.Object = beam
            return beam
        else:
            return self.find_top_beam(beam.this_beam)

    # def find_path_a_b_n(self, beam):
    #     if not hasattr(beam.Proxy, "path_a_b_n"):
    #         return beam.this_beam.Proxy.path_a_b_n
    #     else:
    #         return beam.Proxy.path_a_b_n

    # def find_project_profile_bound_box(self, beam):
    #     if not hasattr(beam.Proxy, "project_profile_bound_box"):
    #         return beam.this_beam.Proxy.project_profile_bound_box
    #     else:
    #         return beam.Proxy.project_profile_bound_box

    def miter_cut(self, beam):
        # get nearest point of the two lines
        top_beam_1 = self.find_top_beam(self.Object.this_beam)
        top_beam_2 = self.find_top_beam(beam)

        path, p_1_a, p_1_b, n1 = top_beam_1.Proxy.path_a_b_n
        path, p_2_a, p_2_b, n2 = top_beam_2.Proxy.path_a_b_n
        arr = [[p_1_a, p_2_a],
               [p_1_a, p_2_b],
               [p_1_b, p_2_a],
               [p_1_b, p_2_b]]

        l_arr = map(norm, arr)
        min_val = min(l_arr)
        min_item = l_arr.index(min_val)

        if min_item in [0, 1]:
            n1 *= -1
        if min_item in [0, 2]:
            n2 *= -1

        bp1, bp2 = arr[min_item]

        # calculating the intersection-point p and face-normal n
        t = n1.cross(n2)
        np_n1, np_n2, np_t, np_bp1, np_bp2 = map(to_np, [n1, n2, t, bp1, bp2])
        np_mat = np.array([np_n1, -np_n2, t])
        np_mat = np_mat.transpose()
        np_rhs = np_bp2 - np_bp1
        l1, l2, l3 = np.linalg.solve(np_mat, np_rhs)
        n = n1 - n2
        n.normalize()
        p = bp1 + n1 * l1 + t * l3 * 0.5

        pol = top_beam_1.Proxy.project_profile_bound_box(n, p)
        face = Part.Face(pol.Wires)
        solid = face.extrude(n1 * 100)
        return self.Object.this_beam.Shape.cut(solid)

    def face_cut(self, beam, face):
        top_beam_1 = self.find_top_beam(self.Object.this_beam)
        top_beam_2 = self.find_top_beam(beam)
        path, p_1_a, p_1_b, n1 = top_beam_1.Proxy.path_a_b_n
        path, p_2_a, p_2_b, n2 = top_beam_2.Proxy.path_a_b_n
        arr = [p_1_a, p_1_b]
        n = face.normalAt(0, 0)
        n.normalize()

        n_dist = lambda p: abs((p - p_2_a).dot(n))
        l_arr = map(n_dist, arr)
        min_val = min(l_arr)
        min_item = l_arr.index(min_val)

        if min_item in [0]:
            n1 *= -1

        # get the face normal and center point

        p = face.CenterOfMass
        pol = top_beam_1.Proxy.project_profile_bound_box(n, p)
        face = Part.Face(pol.Wires)
        solid = face.extrude(n1 * 100)
        return self.Object.this_beam.Shape.cut(solid)

    def shape_cut(self, beam):
        top_beam = self.find_top_beam(beam)
        return self.Object.this_beam.Shape.cut(top_beam.Proxy.outer_shape)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ViewProviderCBeam(ViewProviderBeam):
    def __init__(self, obj):
        ''' Set this object to the proxy object of the actual view provider '''
        obj.Proxy = self
        self.vobj = obj

    def claimChildren(self):
        self.vobj.Object.this_beam.ViewObject.Visibility=False
        return [self.vobj.Object.this_beam]

    def getIcon(self):
        return(ICON_PATH + "beam.svg")


def norm(p):
    return (p[0] - p[1]).Length


def sketch_normal(n, normal_info=None):
    if normal_info.TypeId == 'Sketcher::SketchObject':
        b = App.Vector(0, 0, 1)
        return normal_info.Placement.Rotation.multVec(b)
    else:
        # this is a draft element or something else:
        # no information for the normals of the path
        if abs(n.x) <= abs(n.y):
            if abs(n.x) <= abs(n.z):
                a = App.Vector(1, 0, 0)
            else:
                a = App.Vector(0, 0, 1)
        else:
            if abs(n.y) <= abs(n.z):
                a = App.Vector(0, 1, 0)
            else:
                a = App.Vector(0, 0, 1)
        print(n)
        print(a)
        b = n.cross(a).normalize()
        return b    


def project_to_plane(plane_p, plane_n, p, n):
    '''p + l * n = plane_p + x * plane_n'''
    return p + n * (plane_n.dot(plane_p - p) / (n.dot(plane_n)))


def to_np(app_vec):
    return np.array([app_vec.x, app_vec.y, app_vec.z])

