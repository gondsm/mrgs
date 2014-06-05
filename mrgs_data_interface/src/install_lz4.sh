#!/bin/bash
# This script checks for the existence of lz4 and checks it out if it doesn't exist.
# It is automatically run by CMake, no need to run it manually.
if [ -f ../src/lz4/lz4.h ];
then
   echo "LZ4 already exists."
else
   svn checkout http://lz4.googlecode.com/svn/trunk/ ../src/lz4
fi
