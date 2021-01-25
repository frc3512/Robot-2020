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

os.chdir(f"build/install/frcUserProgramTest/{task_os}x86-64/release")

# Make wrapper script run valgrind
with open("frcUserProgramTest") as input:
    content = input.read()
with open("frcUserProgramTest", "w") as output:
    output.write(content.replace("exec ", "valgrind "))

subprocess.run(["./frcUserProgramTest"], shell=True, check=True)
