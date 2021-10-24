# N-Body Tool Data Format Specification

Input data (stored in `.data` files) used in the visual demos are read at runtime and used to initialize a simulation.

`.data` files only store object starting positions and velocities as well as the bounding box of the objects' initial positions. Other simulation data (like integrator, forceComputer) is specified using #defines.

General naming convention for `.data` files is `ARRANGEMENT_NUMBEROFBODIES.data` (eg. `spiralgalaxy_10k.data`)

There are two types of `.data` files: **2d** and **3d**.

The data for a specific object is terminated by a newline character `\n`. For each object, there are 6 space seperated entries (8 if 3d) that specify the object's mass, radius, initial positon, and then initial velocity. For the vector values, their entries are stored in the order `x y z` (`x y` in the case of 2d).

Example:
```
(zeros_5.data)
m r x y z vx vy vz\n

0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
```
