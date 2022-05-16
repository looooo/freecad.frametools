A frame-module for freecad
---------------------------

## Install:
* Use only with FreeCAD version >= 0.16
* `git clone https://github.com/looooo/freecad_frame.git`
* Link or copy the folder into /freecad/Mod (sudo ln -s (path_to_freecad_frame) (path_to_freecad)/Mod

## Usage

### Create a beam:
* Open freecad
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
  * Select "Beam" command.
  * Select profile.
  * Select path.
  * Enter extents if beams should be longer than path.
  * Enter rotation for profile to turn beam around its path.
  * Press "Create".
  * Select another path or profile to create more beams or press
    "Close" to finish command. 

### Miter cut two beams
  
* Select "Miter Cut" command from Frame and beam WB.
* Select first beam.
* Select second beam which intesects the first beam.
* Press "Cut".
* To cut another beams select them and press "Cut" or press "Close" to
  finish command. 

If one or two beams are selected when "Miter Cut" command activated
they will be preselected as "Beam 1" and "Beam 2". To cut them press
"Cut" button.

### Plane Cut

* Select "Plane Cut" command from Frame and beam WB.
* Select first beam.
* Select face of another beam which intersects the first beam.
* Press "Cut".
* To cut another beams select them and press "Cut" or press "Close" to
  finish command. 

If beam and beam face (in that order) are selected when "Plane Cut"
command activated they will be preselected as "Beam to cut" and "Cut
with". To cut them press "Cut" button.

### Shape Cut

* Select "Shape Cut" command from Frame and beam WB.
* Select first beam.
* Select second beam which intesects the first beam.
* Press "Cut".
* To cut another beams select them and press "Cut" or press "Close" to
  finish command. 

If one or two beams are selected when "Shape Cut" command activated
they will be preselected as "Beam 1" and "Beam 2". To cut them press
"Cut" button.

