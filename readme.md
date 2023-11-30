# Select Point Pair Adapter

A simple tool for wrap4d, transfer xyz coordinate position to barycentric coordinate.

## How to build

```
git submodule init
git submodule upgrade
mkdir build
cd build
cmake ..
cmake --build .
```

Warning: This project uses vcpkg as default package manager. If you have any problem in building, please fix the CMakeLists.txt or your own parameter yourself. I have test the build procedure in both Windows and Linux platforms and the result is good. If you are using windows platform, please don't use Ninja with gcc and use MSVC as default generator. By the way if you hoping build faster you can try Ninja with MSVC on Windows.