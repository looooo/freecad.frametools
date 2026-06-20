import json
import os

import FreeCAD as App

from freecad.frametools import ICON_PATH


def default_constraints():
    return {
        "lengths": [],
        "parallel": [],
        "perpendicular": [],
        "horizontal": [],
        "vertical": [],
    }


def default_lines():
    return []


def parse_lines(raw):
    if not raw:
        return default_lines()
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        return default_lines()
    if not isinstance(data, list):
        return default_lines()
    out = []
    for i, item in enumerate(data):
        if not isinstance(item, dict):
            continue
        try:
            out.append({
                "line": int(item.get("line", i)),
                "u0": float(item["u0"]),
                "v0": float(item["v0"]),
                "u1": float(item["u1"]),
                "v1": float(item["v1"]),
            })
        except (KeyError, TypeError, ValueError):
            continue
    return out


def dump_lines(data):
    return json.dumps(data, indent=2)


def parse_constraints(raw):
    if not raw:
        return default_constraints()
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        return default_constraints()
    out = default_constraints()
    for key in out:
        if key in data and isinstance(data[key], list):
            out[key] = data[key]
    return out


def dump_constraints(data):
    return json.dumps(data, indent=2)


class ImageCalibration(object):

    def __init__(self, obj):
        obj.addProperty(
            "App::PropertyLink", "Image", "Calibration",
            "Bild (ImagePlane oder AlignedImage)")
        obj.addProperty(
            "App::PropertyLink", "Sketch", "Calibration",
            "Aktiver Sketch (Geometrie)")
        obj.addProperty(
            "App::PropertyLink", "InputSketch", "Calibration",
            "Ursprungs-Sketch (Geometrie für jede Kalibrierung)")
        obj.addProperty(
            "App::PropertyString", "Constraints", "Constraints",
            "Bedingungen (JSON)").Constraints = dump_constraints(
                default_constraints())
        obj.addProperty(
            "App::PropertyString", "Lines", "Constraints",
            "Kanten in Bild-UV (parametrisch, L0, L1, …)").Lines = dump_lines(
                default_lines())
        obj.Proxy = self

    def execute(self, obj):
        return

    def onChanged(self, obj, prop):
        if prop in ("Sketch", "InputSketch", "Image"):
            vobj = getattr(obj, "ViewObject", None)
            if vobj is not None:
                vobj.signalChangeIcon()

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class ViewProviderImageCalibration(object):

    def __init__(self, vobj):
        vobj.Proxy = self
        self.Object = vobj.Object
        self.attach(vobj)

    def attach(self, vobj):
        self.ViewObject = vobj
        self.Object = vobj.Object
        from freecad.frametools import image_tools
        image_tools._ensure_calibration_sketch_observer()

    def claimChildren(self):
        from freecad.frametools import image_objects

        obj = self.Object
        children = []
        img = getattr(obj, "Image", None)
        if img is not None:
            children.append(img)
            from freecad.frametools import image_point_alignment as pa
            aligned = pa.find_aligned_image_for_source(img)
            if aligned is not None and aligned not in children:
                children.append(aligned)
                vobj = getattr(img, "ViewObject", None)
                if vobj is not None:
                    vobj.Visibility = False
        sketch = getattr(obj, "Sketch", None)
        if sketch is not None:
            children.append(sketch)
        input_sketch = getattr(obj, "InputSketch", None)
        if input_sketch is not None and input_sketch not in children:
            vobj = getattr(input_sketch, "ViewObject", None)
            if vobj is not None:
                vobj.Visibility = False
            children.append(input_sketch)
        return children

    def setupContextMenu(self, vobj, menu):
        action = menu.addAction("Bedingungen …")
        action.triggered.connect(self.edit_constraints)
        action = menu.addAction("Kalibrieren")
        action.triggered.connect(self.solve)
        action = menu.addAction("Neuen Sketch anlegen")
        action.triggered.connect(self.create_sketch)

    def edit_constraints(self):
        from freecad.frametools import image_tools
        image_tools.show_calibration_constraints_dialog(self.Object)

    def solve(self):
        from freecad.frametools import image_tools
        image_tools.solve_image_calibration(self.Object)

    def create_sketch(self):
        from freecad.frametools import image_tools
        image_tools.create_sketch_for_calibration(self.Object)

    def doubleClicked(self, vobj):
        self.edit_constraints()
        return True

    def getIcon(self):
        return os.path.join(ICON_PATH, "scale_solver.svg")

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        if getattr(self, "ViewObject", None):
            self.attach(self.ViewObject)


def is_image_calibration(obj):
    return (
        getattr(getattr(obj, "Proxy", None), "__class__", None).__name__
        == "ImageCalibration")


def ensure_image_calibration(obj):
    if not is_image_calibration(obj):
        return
    if not obj.Constraints:
        obj.Constraints = dump_constraints(default_constraints())
    if not getattr(obj, "Lines", None):
        obj.Lines = dump_lines(default_lines())
    from freecad.frametools import image_tools
    from freecad.frametools import image_objects

    img = getattr(obj, "Image", None)
    if img is None:
        recovered = image_tools._recover_calibration_image(obj)
        if recovered is not None:
            obj.Image = recovered
