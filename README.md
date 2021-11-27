# nbody-tool
A portable n-body simulation tool for C++.

Cold Collapse                  |  Rotating Stars
:-----------------------------:|:-------------------------:
![](./demo/cold_collapse_demo.gif)  |![](./demo/rotating_demo.gif)

## Features
- Uses multithreading and SIMD + other optimizations.
- Convenient, portable and customizable:
    - Plug and play: Little initial setup required, lots of parameters to fiddle with (Customizable integrators, force computation, etc.).
    - Wide range of applications: From astrodynamics to molecular simulations.

## Quick Start
```
int main() {
    // Create a simulation object
    Simulator sim(
        1,                              // Time passed per simulation step
        1000,                           // Maximum number of bodies
        new VerletIntegrator(),         // Integrator (Verlet is reccommended)
        new Gravitational_BarnesHut(
            1,                          // Barnes-Hut theta parameter
            1,                          // Softening parameter
            Unit::LightYear,            // Distance unit
            Unit::SolarMass,            // Mass unit
            Unit::JulianMillenium       // Time unit
        )
    );

    Rigidbody rb1 = sim.addObject(
                        2,                          // Mass (2 solar masses)
                        7.3603e-8,                  // Radius (in light yearsa)
                        Eigen::Vector3d(0, 0, 0),   // Initial position
                        Eigen::Vector3d(0.5, 0, 0)    // Initial velocity (ly/millenium)
                    )
    Rigidbody rb2 = sim.addObject(4, 1.47206e-7, Eigen::Vector3d(2, 3, 0), Eigen::Vector3d(0, 0.1, 0.2));

    // Moves simulation forward by a simulation step
    sim.step();

    Eigen::Vector3d rb1_pos = sim.rb_pos(rb1);  // Position of rb1
    Eigen::Vector3d rb2_v = sim.rb_v(rb1);      // Velocity of rb1

    return 0;
}
```

## Compiling From Source
CMake and g++ must be installed.\
On Windows, `MSVC` must be used instead of g++.

In the project root,

**Windows (MSVC)**:
```
cmake -S . -B build/ -D NBT_BUILD_TESTS=OFF
cd build/
msbuild ALL_BUILD.vcxproj
```

**Linux**:
```
cmake -S . -B build/ -D NBT_BUILD_TESTS=OFF
cd build/
make all
```

To compile tests, change `BUILD_TESTS=OFF` to `BUILD_TESTS=ON` in the cmake command.\
The resulting static lib `libnbodytool.a` can be found in `build/src/`\
`nbodytool_test` and `benchmark` executables can be found in `build/test/` if compiled.\


TODO: add list of external libs