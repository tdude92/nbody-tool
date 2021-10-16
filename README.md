# nbody-tool
A portable Barnes-Hut n-body simulation tool for C++

## Compiling From Source
CMake and MSVC from Visual Studio Community 2019 must be installed.
In the project root,
```
cmake -S . -B build/ -D BUILDTESTS=OFF -D USECUDA=OFF
cd build/
make all
```
To compile tests, change `BUILDTESTS=OFF` to `BUILDTESTS=ON` in the cmake command.\
Similarly, change `USECUDA=OFF` to `USECUDA=ON` to use CUDA if available.\
The resulting static lib `libnbodytool.a` can be found in `build/src/`\
`nbodytool_test` can be found in `build/test` if compiled.

If using MinGW or MinGW64 on Windows, the process only slightly differs:
```
cmake -S . -B build/ -D BUILDTESTS=OFF -G "MinGW Makefiles"
cd build/
mingw32-make all
```
CUDA cannot be used on Windows without the MSVC compiler (so MinGW can only compile the CPU version of this lib).

TODO update for MSVC
TODO set up Doxygen