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
               shell=True,
               check=True)

# Go to directory for tests debug build
os.chdir(f"build/install/frcUserProgramTest/{task_os}x86-64/debug")

# Write non-interactive gdb commands to a file
with open("gdb-cmds.txt", "w") as output:
    output.write("run\nbt\nquit\n")

# Make wrapper script run gdb
with open("frcUserProgramTest") as input:
    content = input.read()
with open("frcUserProgramTest", "w") as output:
    output.write(content.replace("exec ", "gdb -batch -x gdb-cmds.txt "))

subprocess.run(["./frcUserProgramTest"], shell=True, check=True)
