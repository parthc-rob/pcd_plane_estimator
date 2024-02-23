#!/bin/bash
[ ! -d "./build" ] && mkdir build
cd build

cmake .. && make -j16
#cmake -DCMAKE_BUILD_TYPE=Debug .. && make -j16
