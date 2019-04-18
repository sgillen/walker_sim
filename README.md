Three Link Walker Simulation
============================
This is a collection of code developed while studying planar walkers (https://www.ijsr.net/archive/v2i5/IJSRON2013995.pdf) during my first year at UCSB.


Directory Structure
-------------------
* cg_walker - contains code to simulate a planar compass gait walker

* cg_torso_walker - contains code to simulate a three link walker (compass gait with torso)

* common - bits of code both the walkers need 

* old - older code that I still want available and visible for reference but isn't actually being used by either of the simulators

Compilation
---------------
I use a different version of interp1 in the cg_walker code. I include the .c file used in the repo but you need to compile it yourself using the following matlab commands.
`cd common`
`mex -O -v nakeinterp1.c`
