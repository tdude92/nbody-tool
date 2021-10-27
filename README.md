# nbody-tool
A portable n-body simulation tool for C++.

## Features
- Multithreaded, vectorized, GPU accelerated
- Convenient, portable and customizable:
    - Plug and play: Little initial setup required, lots of parameters to fiddle with (Customizable integrators, force computation, etc.).
    - Wide range of applications: From astrodynamics to molecular simulations.

## Compiling From Source
CMake and g++ must be installed.\
To use CUDA, the toolkit must be installed on the system with `nvcc` in PATH.
On Windows, `MSVC` must be used instead of g++.

In the project root,

**Windows (MSVC)**:
```
cmake -S . -B build/ -D NBT_BUILD_TESTS=OFF -D NBT_USE_CUDA=OFF
cd build/
msbuild ALL_BUILD.vcxproj
```

**Linux**:
```
cmake -S . -B build/ -D NBT_BUILD_TESTS=OFF -D NBT_USE_CUDA=OFF
cd build/
make all
```

To compile tests, change `BUILD_TESTS=OFF` to `BUILD_TESTS=ON` in the cmake command.\
Similarly, change `USE_CUDA=OFF` to `USE_CUDA=ON` to compile for CUDA acceleration.\
The resulting static lib `libnbodytool.a` can be found in `build/src/`\
`nbodytool_test` and `benchmark` executables can be found in `build/test/` if compiled.\

If using MinGW or MinGW64 on Windows, the process only slightly differs:
```
cmake -S . -B build/ -D NBT_BUILD_TESTS=OFF -G "MinGW Makefiles"
cd build/
mingw32-make all
```
CUDA cannot be used on Windows without the MSVC compiler (so MinGW can only compile the CPU version of this lib).

TODO: add list of external libs