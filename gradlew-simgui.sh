#!/bin/bash
set -e

./gradlew simulateExternalCpp
env HALSIM_EXTENSIONS=`find . -type f -name libhalsim_guid.so | sort -r | head -n 1` ./build/install/frcUserProgram/linuxx86-64/debug/frcUserProgram
