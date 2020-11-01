#!/bin/bash
set -e

# Remove old log files
rm -f build/test-results/frcUserProgramTest/linuxx86-64/debug/{*.csv,Robot.log}

./gradlew installFrcUserProgramTestLinuxx86-64DebugGoogleTestExe $@
printf "run\nbt\nquit" > build/install/frcUserProgramTest/linuxx86-64/debug/gdb-cmds.txt
sed -i 's/exec /gdb --command=gdb-cmds.txt /' build/install/frcUserProgramTest/linuxx86-64/debug/frcUserProgramTest
cd build/install/frcUserProgramTest/linuxx86-64/debug && ./frcUserProgramTest
