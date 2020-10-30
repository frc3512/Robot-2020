#!/bin/bash
set -e

# Remove old log files
rm -f build/test-results/frcUserProgramTest/linuxx86-64/debug/{*.csv,Robot.log}

./gradlew simulateExternalCpp
./gradlew installFrcUserProgramTestLinuxx86-64DebugGoogleTestExe $@
sed -i 's/exec /gdb /' build/install/frcUserProgram/linuxx86-64/debug/frcUserProgram
env HALSIM_EXTENSIONS=`find . -type f -name libhalsim_guid.so | sort -r | head -n 1` ./build/install/frcUserProgram/linuxx86-64/debug/frcUserProgram
