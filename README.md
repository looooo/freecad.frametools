# FreeCAD Frame Workbench 
A frame-module for FreeCAD

## Requirements
* FreeCAD >= v0.16
* scipy v?? <----

## Installation
### Automatic Installation
As of v0.17 it's possible to use the built-in FreeCAD [Addon Manager](https://github.com/FreeCAD/FreeCAD-addons#1-builtin-addon-manager) 
to install this workbench.

### Manual Installation
  * `cd ~/.freecad/Mod || mkdir ~/.freecad/Mod && cd ~/.freecad/Mod`
  * `git clone https://github.com/looooo/freecad_frame.git`
  * restart FreeCAD

## Usage

### Create a Beam

* Create new document
* Create profile. Possible sources for profile:
  * Sketch
    * Draw a sketch.
    * Go to Part WB.
    * Select profile.
    * Create Face from profile.
  * "Profile" command in Arch WB allows to insert some predefined
    profiles like RHS. 
  
  The profile should be in XY-plane (this is current limitation). 
* Create a path to make beam. The path can be:
  * Straight lines from any sketch.
  * Straight lines from Draw WB.
  
  Curves can't be used as path for beams.
* Go to Frame and beams WB.
  * Select "Beam" command <img src="/freecad/frametools/icons/beam.svg" width="24" height="24">
  * Select profile.
  * Select path.
  * Enter extents if beams should be longer than path.
  * Enter rotation for profile to turn beam around its path.
  * Press "Create".
  * Select another path or profile to create more beams or press
    "Close" to finish command. 

### Miter cut two beams
  
* Select "Miter Cut" command from Frame and beam WB <img src="/freecad/frametools/icons/beam_miter_cut.svg" width="24" height="24">
* Select first beam.
* Select second beam which intesects the first beam.
* Press "Cut".
* To cut another beams select them and press "Cut" or press "Close" to
  finish command. 

If one or two beams are selected when "Miter Cut" command activated
they will be preselected as "Beam 1" and "Beam 2". To cut them press
"Cut" button.

### Plane Cut

* Select "Plane Cut" command from Frame and beam WB <img src="/freecad/frametools/icons/beam_plane_cut.svg" width="24" height="24">
* Select first beam.
* Select face of another beam which intersects the first beam.
* Press "Cut".
* To cut another beams select them and press "Cut" or press "Close" to
  finish command. 

If beam and beam face (in that order) are selected when "Plane Cut"
command activated they will be preselected as "Beam to cut" and "Cut
with". To cut them press "Cut" button.

### Shape Cut

* Select "Shape Cut" command from Frame and beam WB <img src="/freecad/frametools/icons/beam_shape_cut.svg" width="24" height="24">
* Select first beam.
* Select second beam which intesects the first beam.
* Press "Cut".
* To cut another beams select them and press "Cut" or press "Close" to
  finish command. 

If one or two beams are selected when "Shape Cut" command activated
they will be preselected as "Beam 1" and "Beam 2". To cut them press
"Cut" button.

## Example

Gate leaf creation.

<img src="/example/gate/beams_cut.png">

Prerequisites:

* Made from hollow profile with rounded corners with size 40x20mm and thickness 2mm.
* Internal height between top and bottom bars is 1500mm.
* Horizontal size 1700mm.
* Middle bars added to have decorations inside between top and middle
  bar and bottom and middle bar (not shown due to lack of
  object). Height of decorations is 155mm.
* Diagonal supports added. 

1. Create profile.

   <img src="/example/gate/profile_sketch.png">

   1. Create a new sketch in XY plane.
   2. Repeat drawing from picture above. Outer rounding radius is 3mm
      and inner rounding radius is 2mm.
   3. Close sketch.
   4. Select Part Workbench and convert sketch to face ("Make face from
      wires" command).

2. Create sketch for beam axes. Since this is gate leaf the sketch
   should be created, for example, in XZ plane.
   
   <img src="/example/gate/beams_sketch.png">
   
   This will require a little bit more work because some measurements
   give outer sizes, some give inner and no one give axes. The
   diagonal supports are drawn according to other beams. To achieve
   correct lines construction geometry should be used for everything
   that is not beam in final design like outer edge of gate. 

3. Create outer beams. Since they should fully intersect the "Extent
   1" and "Extent 2" should have value 20mm at least. Please don't
   enter very big values because some extra parts can be left after
   cutting.
   
   <img src="/example/gate/outer_beams.png">
   
4. Create inner beams. They axes long enough to get full intersection
   with other beams so the extents can be left as is.
   
   <img src="/example/gate/inner_beams.png">
   
5. Miter cut outer beams.

   <img src="/example/gate/outer_beams_cut.png">
   
   Please note that *every* cut operation creates a new beam so if
   beam needs to be cut twice it should be reselected after the
   first cut. Removing cut beam can break other cut beams if it is
   used in the other beam cut operation.

6. Plane cut inner beams by outer beams inner planes. Plane cut
   diagonals.

   <img src="/example/gate/beams_cut.png">
   
7. Select TechDraw Workbench

   <img src="/example/gate/drawing.png">
   
   1. Insert page from template and select A4 Portrait template.
   2. Go to the 3D view and select "Front View" to see front side. The
      TechDraw inserts view in current active projection.
   3. Select Main Diagonal.
   4. Go to the inserted page and press "Insert view". The page will
      contain huge page placed as viewed.
   5. Select view in tree and set scale to 0.1. This makes beam
      completely inside drawing.
   6. The TechDraw don't allow to simple rotations like "make this
      line vertical" so view should be rotated by -55.582°
      (measured as angle between vertical border and angled one). This
      make beam straight vertical on drawing.
   7. Add dimensions to overall length and how to cut corners.
   
   Repeat steps above for every unique beam. Fill fields below as
   needed. This gives the set of drawing suitable to create all parts
   of gate leaf. 
   
   To have drawing with guide to assemble it full view (with all part
   selected) should be inserted on a new page. Welding annotations
   will describe how to join parts.
   
## Feedback
Offer feedback and discuss this workbench in the [dedicated FreeCAD forum thread]()

## Known Limitations

## Developer
[@looooo](https://github.com/looooo)

## License
GNU Lesser General Public License v2.1
