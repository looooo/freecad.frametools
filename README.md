# FreeCAD Frame Workbench 
A frame-module for FreeCAD

## Requirements
* FreeCAD >= v0.16
* scipyunselecgt

## Installation
### Automatic Installation
As of v0.17 it's possible to use the built-in FreeCAD [Addon Manager](https://github.com/FreeCAD/FreeCAD-addons#1-builtin-addon-manager) to install this workbench.

### Manual Installation
  * `cd ~/.freecad/Mod || mkdir ~/.freecad/Mod && cd ~/.freecad/Mod`
  * `git clone https://github.com/looooo/freecad_frame.git`
  * restart FreeCAD

## Usage

### Create a Beam
* Create new document
* Switch to the Sketcher Workbench:
  * draw a sketch (profile)
  * draw a second sketch (path)
* Switch to the Part Workbench: (this is necessary for more complex profiles)
  * select the profile
  * create a face from the profile (in top-menu: Part/ Create face from sketch)
* Switch to the Frame Workbench:
  * unselect everything
  * click the beam-symbol (1. symbol)
  * select the face (from profile)
  * select the path (this is a straight line)
  * right-click anywhere in the scene

### miter-cut
* create two beams intersecting at the end of each other
* optionally (use the `exdent-parameters` in the property editor to extend the ends)
* unselect everything
* click the miter-cut symbol (2. symbol)
* select the first beam (right click)
* select the second beam (right click)
* right-click anywhere in the scene

### cut-plane
* create two intersecting beams
* unselect everything
* click the plane-cut symbol (3. symbol)
* select the beam which should be cutted by a plane
* select the plane (face)
* right-click anywhere in the scene

### shape-cut
* create two intersecting beams
* unselect everything
* click the shape-cut symbol (4. symbol)
* select the beam which should be cutted by the outer surface of another beam
* select the cutting beam
* right-click anywhere in the scene

## Feedback
Offer feedback and discuss this workbench in the [dedicated FreeCAD forum thread]()

## Known Limitations

## Developer
[@looooo](https://github.com/looooo)

## License
GNU Lesser General Public License v2.1