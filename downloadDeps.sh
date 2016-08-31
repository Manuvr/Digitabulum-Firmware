#!/bin/bash
#
# This script is meant to go fetch the most recent versions of various libraries that
#   ManuvrOS has been written against. None of this is strictly required for a basic build,
#   but most real-world applications will want at least one of them.

# Make the lib directory...
mkdir lib

# CBOR...
# Note that we do special-handling here to make the build-process smoother...
rm -rf lib/cbor-cpp
git clone https://github.com/naphaso/cbor-cpp.git lib/cbor-cpp
ln -s `pwd`/lib/cbor-cpp/src/ lib/cbor-cpp/include

# Return...
cd ..
