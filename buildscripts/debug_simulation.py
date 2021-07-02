#!/usr/bin/env python3

import os
import platform
import subprocess

if platform.system() == "Linux":
    task_os = "linux"
    debugger = "gdb"
elif platform.system() == "Darwin":
    task_os = "osx"
    debugger = "lldb"
elif platform.system() == "Windows":
    task_os = "windows"
    debugger = "windbg"

# Build simulation
subprocess.run(["./gradlew simulateExternalCpp"], shell=True, check=True)

# Configure HALSIM extension
extensions = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk(".")
    for f in fn
    if "halsim_guid" in f
]
os.environ["HALSIM_EXTENSIONS"] = os.path.abspath(
    list(reversed(sorted(extensions)))[0])

# Go to directory for simulation debug build
os.chdir(f"build/install/frcUserProgram/{task_os}x86-64/debug")

# Make wrapper script run debugger
with open("frcUserProgram") as input:
    content = input.read()
with open("frcUserProgram", "w") as output:
    output.write(content.replace("exec ", f"{debugger} "))

subprocess.run(["./frcUserProgram"], shell=True, check=True)
