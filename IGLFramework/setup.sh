#!/usr/bin/env bash
git clone https://github.com/libigl/libigl.git 3rdparty/libigl
cd 3rdparty/libigl/external/nanogui
git submodule update --init --recursive .
cd ../../../..
mkdir build
cd build
cmake ..
make -j8
IGLFramework/iglFramework ../3rdparty/libigl/tutorial/shared/bunny.off
