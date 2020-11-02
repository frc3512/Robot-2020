#!/bin/bash
set -e

./buildscripts/remove-logs.sh

./gradlew installFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe $@
sed -i 's/exec /valgrind /' build/install/frcUserProgramTest/linuxx86-64/release/frcUserProgramTest
cd build/install/frcUserProgramTest/linuxx86-64/release && ./frcUserProgramTest