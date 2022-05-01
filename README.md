# 3D experiment C++ ver

# Necessary
- C++ support
- Visual Studio 2019
- OpenCV library

# Generate
```
git clone https://github.com/isl-org/Open3D
mkdir build
cd build
cmake -G "Visual Studio 16 2019" -A x64 -DCMAKE_INSTALL_PREFIX="<open3d_install_directory>" 
cmake --build . --config Release --target ALL_BUILD
```

# Debug
- Open Open3D.sln 
- For example, open examples/cpp/GeneralizedICP peoperty
- add opencv_path/build/include to Additional Include Directories
- add opencv_path/build/x64/vc15/lib to Additional Library Directories
- add opencv_world411d.lib and opencv_world411.lib to Additional Dependency File
- replace examples/cpp/GeneralizedICP/Source Files/GeneralizedICP.cpp with this file
- You can Debug this project
