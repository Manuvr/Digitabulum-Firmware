#!/bin/bash
#
# This script is meant to go fetch the most recent versions of various libraries that
#   ManuvrOS has been written against. None of this is strictly required for a basic build,
#   but most real-world applications will want at least one of them.

# Manuvr
#rm -rf lib/ManuvrOS
git clone https://github.com/jspark311/ManuvrOS lib/ManuvrOS
