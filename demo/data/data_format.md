# N-Body Tool Data Format Specification

Input data (stored in `.data` files) used in the visual demos are read at runtime and used to initialize a simulation.

`.data` files only store object starting positions and velocities and the units used in the simulation. Other simulation data (like integrator, forceComputer) is specified using #defines.

General naming convention for `.data` files is `ARRANGEMENT_NUMBEROFBODIES.data` (eg. `spiralgalaxy_10k.data`)

There are two types of `.data` files: **2d** and **3d**.

The first line in the file contains three space seperated doubles specifying the conversion factor from an SI unit to the specific unit used in the simulation (eg. 1km = 1000m). In order, they are length, mass, time.

The data for a specific object is terminated by a newline character `\n`. For each object, there are 6 space seperated entries (8 if 3d) that specify the object's mass, radius, initial positon, and then initial velocity. For the vector values, their entries are stored in the order `x y z` (`x y` in the case of 2d).

Example:
```
(zeros_5.data)
m r x y z vx vy vz\n

unit_l unit_m unit_t
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
0 0 0 0 0 0 0 0\n
```
