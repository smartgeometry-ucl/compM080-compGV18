cmake_minimum_required(VERSION 3.6)
project(acq16)

set(CMAKE_CXX_STANDARD 11)

# Compile type: Release, Debug, RelWithDebInfo
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Where external libraries are
set(THIRD_PARTY_DIR 3rdparty)

# definitions
add_definitions(-DIGL_VIEWER_WITH_NANOGUI)

# Eigen
find_package(Eigen3 REQUIRED HINTS /usr/local)
include_directories(${EIGEN3_INCLUDE_DIR})

# IGL
include_directories(${THIRD_PARTY_DIR}/libigl/include)
include_directories(${THIRD_PARTY_DIR}/libigl/install/include)
link_directories(${THIRD_PARTY_DIR}/libigl/lib)

# NanoGui
include_directories(${THIRD_PARTY_DIR}/libigl/external/nanogui/install/include)
include_directories(${THIRD_PARTY_DIR}/libigl/external/nanogui/ext/nanovg/src/)

# NanoFLANN
include_directories(${THIRD_PARTY_DIR}/nanoflann/include)

# List of source files
set(SOURCE_FILES
    main.cpp
    )

# Create program
add_executable(acq16 ${SOURCE_FILES})

# Link dependency libraries
target_link_libraries(acq16 embree nanogui glfw3 X11 Xrandr Xinerama Xi Xxf86vm Xcursor GLEW GL dl pthread)

# Optional to compile IGL: http://libigl.github.io/libigl/optional/
# cd libigl
# mkdir lib
# cd lib
# cmake -DCMAKE_BUILD_TYPE=Release -DLIBIGL_WITH_NANOGUI=YES -DLIBIGL_WITH_ANTTWEAKBAR=NO -DLIBIGL_WITH_CGAL=NO ../optional

