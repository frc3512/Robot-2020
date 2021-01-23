#!/usr/bin/env python3

import os
import platform
import subprocess

if platform.system() == "Linux":
    task_os = "linux"
elif platform.system() == "Darwin":
    task_os = "osx"
elif platform.system() == "Windows":
    task_os = "windows"

# Build tests
subprocess.run([
    "./gradlew installFrcUserProgramTest" + task_os.capitalize() +
    "x86-64DebugGoogleTestExe"
],
               shell=True)

# Go to directory for tests debug build
os.chdir(f"build/install/frcUserProgramTest/{task_os}x86-64/debug")

# Make wrapper script run gdb
with open("frcUserProgramTest") as input:
    content = input.read()
with open("frcUserProgramTest", "w") as output:
    output.write(content.replace("exec ", "gdb "))

subprocess.run(["./frcUserProgramTest"], shell=True)
