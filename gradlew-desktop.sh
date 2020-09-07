#!/bin/bash

./gradlew simulateExternalCpp
if [ $? -eq 1 ]; then
  exit 1
fi

env HALSIM_EXTENSIONS=`find . -type f -name libhalsim_guid.so | sort -r | head -n 1` ./build/install/frcUserProgram/linuxx86-64/debug/frcUserProgram
