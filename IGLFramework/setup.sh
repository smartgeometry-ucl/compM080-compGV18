#!/usr/bin/env bash

# Clone libIGL library from GitHub
git clone https://github.com/libigl/libigl.git 3rdparty/libigl
# Enter NanoGui external subdirectory of libIGL
cd 3rdparty/libigl/external/nanogui
# Clone NanoGui
git submodule update --init --recursive .
# Go back up to setup.sh's directory
cd ../../../..
# Create build directory
mkdir build
# Enter build directory
cd build

# Make sure CMake is in OSX path
if [[ $OSTYPE == darwin* ]];
then
    export PATH="/Applications/CMake.app/Contents/bin":${PATH}
fi

# Configure IGLFramework project using ../CMakeLists.txt
cmake ..
# Make IGLFramework project using the generated Makefile
make -j8

# Generate XCode project file
if [[ $OSTYPE == darwin* ]];
then
    cmake -G "Xcode" ..
fi

# Run executable generated
./iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off
