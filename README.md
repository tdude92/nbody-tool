# nbody-tool
A portable Barnes-Hut n-body simulation tool for C++

## Compiling From Source
Make sure to have CMake and g++ installed.
In the project root,
```
cmake -S . -B build/ -D BUILDTESTS=OFF
cd build/
make all
```
To compile tests, change `BUILDTESTS=OFF` to `BUILDTESTS=ON` in the cmake command.\
The resulting static lib `libnbodytool.a` can be found in `build/src/`\
`nbodytool_test` can be found in `build/test` if compiled.

If using MinGW or MinGW64 on Windows, the process only slightly differs:
```
cmake -S . -B build/ -D BUILDTESTS=OFF -G "MinGW Makefiles"
cd build/
mingw32-make all
```