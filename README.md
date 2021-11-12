# nbody-tool
A portable n-body simulation tool for C++.

Cold Collapse                  |  Rotating Stars
:-----------------------------:|:-------------------------:
![](./demo/cold_collapse_demo.gif)  |![](./demo/rotating_demo.gif)

## Features
- Multithreaded, vectorized, optimized
- Convenient, portable and customizable:
    - Plug and play: Little initial setup required, lots of parameters to fiddle with (Customizable integrators, force computation, etc.).
    - Wide range of applications: From astrodynamics to molecular simulations.

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