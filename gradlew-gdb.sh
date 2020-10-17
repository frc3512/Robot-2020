#!/bin/bash

# Remove old log files
rm -f build/test-results/frcUserProgramTest/linuxx86-64/debug/{*.csv,Robot.log}

./gradlew installFrcUserProgramTestLinuxx86-64DebugGoogleTestExe $@
sed -i 's/exec /gdb /' build/install/frcUserProgramTest/linuxx86-64/debug/frcUserProgramTest
cd build/install/frcUserProgramTest/linuxx86-64/debug && ./frcUserProgramTest
