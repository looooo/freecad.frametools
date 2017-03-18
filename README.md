a frame-module for freecad
---------------------------

* install:
  * use only with FreeCAD version >= 0.16
  * git clone https://github.com/looooo/freecad_frame.git
  * link or copy the folder into /freecad/Mod (sudo ln -s (path_to_freecad_frame) (path_to_freecad)/Mod

## usage

* create a beam:
  * open freecad
  * create new document
  * go to Sketcher WB:
    * draw a sketch (profile)
    * draw a second sketch (path)
  * go to Part WB: (this is necesarry for more complex profiles)
    * select the profile
    * create a face from the profile (in top-menu: Part/ Create face from sketch)
  * go to frame WB:
    * unselect everything
    * click the beam-symbol (1. symbol)
    * select the face (from profile)
    * select the path (this is a straight line)
    * right-click anywhere in the scene

* miter-cut
  * create two beams intersecting at the end of each other
  * optionally (use the exdent-parameters in the property editor to extend the ends)
  * unselecgt everything
  * click the miter-cut symbol (2. symbol)
  * select the first beam (right click)
  * select the second beam (right click)
  * right-click anywhere in the scene

* cut-plane
  * create two intersecting beams
  * unselect everything
  * click the plane-cut symbol (3. symbol)
  * select the beam which should be cutted by a plane
  * select the plane (face)
  * right-click anywhere in the scene

* shape-cut
  * create two intersecting beams
  * unselect everything
  * click the shape-cut symbol (4. symbol)
  * select the beam which should be cutted by the outer surface of another beam
  * select the cutting beam
  * right-click anywhere in the scene

