Three Link Walker Simulation
============================
This is a collection of code developed while studying three link walkers (https://www.ijsr.net/archive/v2i5/IJSRON2013995.pdf) during my first few weeks at UCSB


Directory Structure
-------------------
* walker - Diretory for intial walker simulation, as described in the link given above, heavily inspired by the acrobot example
 
* walker2 - Same idea as walker but with a slightly modified model, see the comments for more info

* two_link - Does out the forward and reverse kinematics for a two link arm

* katie_examples - Code taken from some of katie's courses, a useful reference

* acrobot files - Katie's code for simulation of the acrobot, walker and walker2 follow the example set here

* cg_torso_files - Katie's code (with some very small modifications) for simulation of a three link walker with a torso

* cg_walker - poorly named, but this is Sean's refactor of cg_torso_files, does the same thing (for now) but it's cleaned up and everything is in a class  


Important Note
---------------
I use a different version of interp1 in the cg_walker code. I include the .c file used in the repo but you need to compile it yourself using the following commands from the matlab command line.

`cd walker/cg_walker/`

`mex -O -v nakeinterp1.c`

That first command might not be exactly right depending on your settings, the important part is that you are in the cg_walker directory when you execute mex 