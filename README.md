a frame-module for freecad
---------------------------

* install:
  * use only with FreeCAD version >= 0.16
  * git clone https://github.com/looooo/freecad_frame.git
  * link or copy the folder into /freecad/Mod (sudo ln -s (path_to_freecad_frame) (path_to_freecad)/Mod


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
    * select the beam-symbol (1.)
    * select the face (from profile)
    * select the path

