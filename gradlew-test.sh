#!/bin/bash

# Remove old log files
rm -f build/test-results/frcUserProgramTest/linuxx86-64/release/{*.csv,Robot.log}

./gradlew runFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe
