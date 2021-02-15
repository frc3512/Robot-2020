#!/usr/bin/env python3

import platform
import shutil

if platform.system() == "Linux":
    task_os = "linux"
elif platform.system() == "Darwin":
    task_os = "osx"
elif platform.system() == "Windows":
    task_os = "windows"

shutil.copy(f"build/compile_commands/{task_os}x86-64/compile_commands.json",
            "compile_commands.json")
