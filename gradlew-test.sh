#!/bin/bash

# Remove old log files
rm -f *.csv Robot.log

./gradlew runFrcUserProgramTestLinuxx86-64ReleaseGoogleTestExe
