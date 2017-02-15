#!/usr/bin/env bash

# Clone libIGL library from GitHub
if [ ! -e 3rdparty/libigl ];
then
    git clone https://github.com/libigl/libigl.git 3rdparty/libigl
fi
# Enter NanoGui external subdirectory of libIGL
cd 3rdparty/libigl
# Clone NanoGui
git submodule update --init --recursive -- external/nanogui
# Go back up to setup.sh's directory
cd ../..
# Create build directory
if [ ! -e build ]; then
    mkdir build
fi
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
if [ -e iglFramework ];
then
    echo -e "\n\nRunning ./iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off\n"
    ./iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off
elif [ -e IGLFramework/iglFramework ];
then
    echo -e "\n\nRunning IGLFramework/iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off\n"
    IGLFramework/iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off
else
    echo "Could not find any executable to run..."
fi
