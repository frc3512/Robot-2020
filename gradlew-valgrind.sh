#!/bin/bash

# Remove old log files
rm -f build/test-results/frcUserProgramTest/linuxx86-64/release/{*.csv,Robot.log}

./gradlew installFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe $@
sed -i 's/exec /valgrind /' build/install/frcUserProgramTest/linuxx86-64/release/frcUserProgramTest
cd build/install/frcUserProgramTest/linuxx86-64/release && ./frcUserProgramTest
